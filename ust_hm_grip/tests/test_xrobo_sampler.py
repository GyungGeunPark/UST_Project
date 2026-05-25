"""XRoboSampler unit tests (research/47 §8.2).

These tests run WITHOUT a real xrobotoolkit_sdk install by monkey-patching
``sys.modules['xrobotoolkit_sdk']`` with a fake module that returns
deterministic data.  This lets CI verify the snapshot dict layout and
the lifecycle without hardware.
"""

from __future__ import annotations

import sys
import time
import types
import threading

import numpy as np
import pytest


class _FakeXRT:
    """Stand-in for the xrobotoolkit_sdk module."""

    def __init__(self) -> None:
        self.init_called = False
        self.close_called = False
        # Default deterministic data (XR-space xyzw poses + analog values)
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.left_grip = 0.0
        self.right_grip = 0.0
        self.left_pose = np.array([0.1, 1.2, 0.5, 0.0, 0.0, 0.0, 1.0])
        self.right_pose = np.array([0.2, 1.2, 0.5, 0.0, 0.0, 0.0, 1.0])
        self.head_pose = np.array([0.0, 1.7, 0.0, 0.0, 0.0, 0.0, 1.0])

    def init(self):
        self.init_called = True

    def close(self):
        self.close_called = True

    def get_headset_pose(self):
        return self.head_pose

    def get_left_controller_pose(self):
        return self.left_pose

    def get_right_controller_pose(self):
        return self.right_pose

    def get_left_trigger(self):
        return self.left_trigger

    def get_right_trigger(self):
        return self.right_trigger

    def get_left_grip(self):
        return self.left_grip

    def get_right_grip(self):
        return self.right_grip

    def get_time_stamp_ns(self):
        return int(time.time_ns())

    def is_body_data_available(self):
        return False

    def get_body_joints_pose(self):
        return np.zeros((24, 7))


@pytest.fixture
def fake_xrt(monkeypatch):
    """Install a fake xrobotoolkit_sdk module."""
    fake = _FakeXRT()
    # Force re-import path
    sys.modules.pop("xrobotoolkit_sdk", None)
    fake_module = types.ModuleType("xrobotoolkit_sdk")
    for attr in dir(fake):
        if not attr.startswith("_"):
            setattr(fake_module, attr, getattr(fake, attr))
    sys.modules["xrobotoolkit_sdk"] = fake_module
    yield fake
    sys.modules.pop("xrobotoolkit_sdk", None)


@pytest.fixture
def sampler(fake_xrt):
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import XRoboSampler
    s = XRoboSampler(rate_hz=200.0, debug=False)
    yield s
    if s.is_running:
        s.stop()


def _wait_for_frames(sampler, n: int, timeout: float = 2.0) -> bool:
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if sampler.frame_count >= n:
            return True
        time.sleep(0.01)
    return False


def test_sampler_lifecycle(sampler, fake_xrt):
    """start() -> running, stop() -> idle."""
    assert not sampler.is_running
    sampler.start()
    assert sampler.is_running
    assert fake_xrt.init_called
    sampler.stop()
    assert not sampler.is_running
    assert fake_xrt.close_called


def test_sampler_emits_snapshot_dict_shape(sampler, fake_xrt):
    """Snapshot dict must have the same top-level keys as SteamVRSampler."""
    sampler.start()
    assert _wait_for_frames(sampler, 3)
    snap = sampler.snapshot()
    for key in ("timestamp", "hmd", "trackers", "hands", "controllers", "frame_count"):
        assert key in snap, f"missing snapshot key: {key}"
    assert isinstance(snap["controllers"], dict)
    assert "left" in snap["controllers"]
    assert "right" in snap["controllers"]


def test_sampler_controller_pose_in_il_frame(sampler, fake_xrt):
    """XR (0.1, 1.2, 0.5) -> IL (-0.5, -0.1, +1.2).

    Mapping (proper rotation, det=+1, OpenXR LOCAL):
        x_il = -z_xr = -0.5
        y_il = -x_xr = -0.1
        z_il = +y_xr = +1.2
    """
    sampler.start()
    assert _wait_for_frames(sampler, 3)
    snap = sampler.snapshot()
    lpose = snap["controllers"]["left"]["pose"]
    np.testing.assert_allclose(lpose["pos"], (-0.5, -0.1, 1.2), atol=1e-6)


def test_sampler_trigger_grip_passes_through(sampler, fake_xrt):
    """Analog values forwarded into snapshot."""
    fake_xrt.left_trigger = 0.7
    fake_xrt.left_grip = 0.3
    fake_xrt.right_trigger = 0.0
    fake_xrt.right_grip = 0.95
    sampler.start()
    assert _wait_for_frames(sampler, 5)
    snap = sampler.snapshot()
    btn_l = snap["controllers"]["left"]["buttons"]
    btn_r = snap["controllers"]["right"]["buttons"]
    assert btn_l["trigger"] == pytest.approx(0.7, abs=1e-6)
    assert btn_l["grip"] == pytest.approx(0.3, abs=1e-6)
    assert btn_r["trigger"] == pytest.approx(0.0, abs=1e-6)
    assert btn_r["grip"] == pytest.approx(0.95, abs=1e-6)


def test_sampler_analog_clamping(sampler, fake_xrt):
    """Out-of-range analog values are clamped to [0, 1]."""
    fake_xrt.left_trigger = -0.5
    fake_xrt.right_grip = 1.5
    sampler.start()
    assert _wait_for_frames(sampler, 5)
    snap = sampler.snapshot()
    assert snap["controllers"]["left"]["buttons"]["trigger"] == 0.0
    assert snap["controllers"]["right"]["buttons"]["grip"] == 1.0


def test_sampler_frame_count_monotonic(sampler, fake_xrt):
    """frame_count strictly increases."""
    sampler.start()
    assert _wait_for_frames(sampler, 10)
    f1 = sampler.frame_count
    time.sleep(0.1)
    f2 = sampler.frame_count
    assert f2 > f1


def test_sampler_threadsafe_snapshot(sampler, fake_xrt):
    """Concurrent snapshot() from multiple threads."""
    sampler.start()
    assert _wait_for_frames(sampler, 5)
    results = []
    lock = threading.Lock()

    def reader():
        for _ in range(50):
            snap = sampler.snapshot()
            with lock:
                results.append(snap)

    threads = [threading.Thread(target=reader) for _ in range(3)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=2.0)
    assert len(results) == 150
    for snap in results:
        assert "controllers" in snap
        assert "left" in snap["controllers"]


# ── Face buttons (13th-bis: A/B/X/Y for recalibration trigger) ────────
def test_sampler_face_buttons_present_with_default_zero(sampler, fake_xrt):
    """When the fake SDK doesn't expose get_*_button, sampler emits
    A/B (right) and X/Y (left) as False — graceful fallback."""
    sampler.start()
    assert _wait_for_frames(sampler, 3)
    snap = sampler.snapshot()
    rbtn = snap["controllers"]["right"]["buttons"]
    lbtn = snap["controllers"]["left"]["buttons"]
    # Right controller exposes A (lower) + B (upper)
    assert "a" in rbtn and "b" in rbtn
    assert rbtn["a"] is False and rbtn["b"] is False
    # Left controller exposes X (lower) + Y (upper)
    assert "x" in lbtn and "y" in lbtn
    assert lbtn["x"] is False and lbtn["y"] is False


def test_sampler_a_button_pressed_when_sdk_returns_true(monkeypatch, fake_xrt):
    """When ``get_A_button`` returns True, snapshot's right.buttons.a = True."""
    # Attach a get_A_button hook to the fake SDK
    fake_module = sys.modules["xrobotoolkit_sdk"]
    setattr(fake_module, "get_A_button", lambda: True)
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import XRoboSampler
    s = XRoboSampler(rate_hz=200.0, debug=False)
    s.start()
    try:
        assert _wait_for_frames(s, 3)
        snap = s.snapshot()
        assert snap["controllers"]["right"]["buttons"]["a"] is True
        # B not set (no get_B_button hook) → False fallback
        assert snap["controllers"]["right"]["buttons"]["b"] is False
    finally:
        s.stop()


def test_sampler_xrt_missing_raises(monkeypatch):
    """Missing xrobotoolkit_sdk raises with actionable error."""
    sys.modules.pop("xrobotoolkit_sdk", None)
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import XRoboSampler

    real_import = (
        __builtins__["__import__"] if isinstance(__builtins__, dict)
        else __builtins__.__import__
    )

    def _no_xrt(name, *args, **kwargs):
        if name == "xrobotoolkit_sdk":
            raise ImportError("simulated")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", _no_xrt)

    s = XRoboSampler()
    with pytest.raises(RuntimeError, match="xrobotoolkit_sdk is not installed"):
        s.start()


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(pytest.main([__file__, "-v"]))
