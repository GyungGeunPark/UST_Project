"""Regression tests for the 9.38 openvr.init() watchdog timeout.

The watchdog converts a silent infinite hang (when SteamVR is not running
or the HMD provider has not initialised) into a TimeoutError with
actionable diagnostic hints.  See memory.md §10.45 / gotcha #30.

These tests use a fake openvr module so they can run in CI without an
actual SteamVR install.
"""

from __future__ import annotations

import sys
import time

import pytest

# Import the helper directly — keeps the test cheap (no SteamVRSampler
# instantiation needed) and pinpoints regressions to the watchdog itself.
from ust_ws.ust_hm_glove.teleop.vr_sampler import (
    _DEFAULT_OPENVR_INIT_TIMEOUT_SEC,
    _init_openvr_with_timeout,
)


class _FakeOpenVR:
    """Minimal stand-in for the openvr module."""

    VRApplication_Other = 1

    def __init__(self, mode: str):
        self._mode = mode

    def init(self, app_type):  # noqa: D401 — matches openvr.init signature
        if self._mode == "ok":
            return "FAKE_VR_SYSTEM"
        if self._mode == "raise":
            raise RuntimeError("simulated openvr init failure")
        if self._mode == "hang":
            time.sleep(60)  # exceeds any sane test timeout
            return "should_never_return"
        raise ValueError(f"unknown mode {self._mode!r}")


@pytest.fixture
def restore_openvr():
    """Snapshot/restore sys.modules['openvr'] around each test."""
    saved = sys.modules.get("openvr")
    try:
        yield
    finally:
        if saved is None:
            sys.modules.pop("openvr", None)
        else:
            sys.modules["openvr"] = saved


def test_default_timeout_is_30s() -> None:
    """The default must remain 30s — see CLI flag --sampler_init_timeout."""
    assert _DEFAULT_OPENVR_INIT_TIMEOUT_SEC == 30.0


def test_success_path_returns_handle(restore_openvr) -> None:
    sys.modules["openvr"] = _FakeOpenVR("ok")
    t0 = time.perf_counter()
    handle = _init_openvr_with_timeout(timeout_sec=2.0)
    elapsed = time.perf_counter() - t0
    assert handle == "FAKE_VR_SYSTEM"
    # Should be near-instant — daemon thread overhead only
    assert elapsed < 0.5, f"success path too slow: {elapsed:.2f}s"


def test_propagates_exceptions(restore_openvr) -> None:
    """Exceptions from openvr.init must surface on the caller thread verbatim."""
    sys.modules["openvr"] = _FakeOpenVR("raise")
    with pytest.raises(RuntimeError, match="simulated openvr init failure"):
        _init_openvr_with_timeout(timeout_sec=2.0)


def test_timeout_fires_with_diagnostic(restore_openvr) -> None:
    """When openvr.init blocks past timeout, raise TimeoutError with hints."""
    sys.modules["openvr"] = _FakeOpenVR("hang")
    t0 = time.perf_counter()
    with pytest.raises(TimeoutError) as excinfo:
        _init_openvr_with_timeout(timeout_sec=1.0)
    elapsed = time.perf_counter() - t0
    # Tolerance: must fire close to the requested timeout
    assert 0.95 <= elapsed <= 1.6, f"timeout fired at unexpected time: {elapsed:.2f}s"
    msg = str(excinfo.value)
    # Diagnostic message must reference the 4 root causes for actionability
    assert "SteamVR" in msg
    assert "PICO Connect" in msg or "Steam Link" in msg or "Virtual Desktop" in msg
    assert "diagnose_pico_connect" in msg


def test_timeout_clamps_to_minimum(restore_openvr) -> None:
    """SteamVRSampler.__init__ clamps timeout to min 1s — verify the helper
    accepts (and respects) any positive float since the clamp is in __init__,
    not the helper itself."""
    sys.modules["openvr"] = _FakeOpenVR("hang")
    with pytest.raises(TimeoutError):
        _init_openvr_with_timeout(timeout_sec=0.5)
