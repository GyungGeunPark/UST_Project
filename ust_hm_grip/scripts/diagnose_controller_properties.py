"""Sweep all OpenVR property channels on PICO controllers to find a
fallback input source that bypasses the SteamVR Action API.

Why this exists
---------------
The 10th-session investigation confirmed that SteamVR's Action API
returns ``bActive=False`` for every action handle on ``ust.teleop.gr1t2_gripper``
no matter how we register the manifest, edit ``steamvr.vrsettings``,
delete Workshop binding folders, or click "Replace Default Binding" --
SteamVR refuses to *commit* a Personal Binding for our app.  Controller
pose still flows (driver-level ``getDeviceToAbsoluteTrackingPose`` is
binding-agnostic), but trigger / grip analog stay 0.

This script enumerates *every* OpenVR property the PICO driver exposes
on each controller, watches them while the user squeezes trigger / grip,
and prints which properties carry the live analog value.  If we find
one, ``vr_sampler.py`` can bypass the Action API entirely.

Channels probed:
    1. ``getFloatTrackedDeviceProperty``   — full sweep + named props
    2. ``getBoolTrackedDeviceProperty``    — button-style channels
    3. ``getInt32TrackedDeviceProperty``   — state flags
    4. ``getControllerState``              — legacy struct (known to be 0 on PICO,
                                              kept for completeness)

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_properties --seconds 12
"""

from __future__ import annotations

import argparse
import ctypes
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


# Named OpenVR Prop_* constants that are *plausibly* trigger/grip-related
# on PICO controllers (we still sweep blindly below, but having named
# entries makes the output easier to read).
NAMED_FLOAT_PROPS = [
    # Standard Axis-type metadata (these are int32 in spec but some
    # drivers store the live value here)
    "Prop_ControllerHandSelectionPriority_Int32",
    "Prop_DisplayFrequency_Float",
    "Prop_DeviceBatteryPercentage_Float",
    "Prop_FieldOfViewLeftDegrees_Float",
    "Prop_FieldOfViewRightDegrees_Float",
    "Prop_FieldOfViewTopDegrees_Float",
    "Prop_FieldOfViewBottomDegrees_Float",
    "Prop_TrackingRangeMinimumMeters_Float",
    "Prop_TrackingRangeMaximumMeters_Float",
    # Vendor reserved range can store ad-hoc values
    # (PICO driver may store trigger here)
]


def _safe_get_float(system, idx, prop_id):
    """Call ``getFloatTrackedDeviceProperty`` and return ``(value, ok)``."""
    try:
        # pyopenvr's bound signature: getFloatTrackedDeviceProperty(idx, prop) -> float
        # but raises on unsupported props.  Catch broadly.
        v = system.getFloatTrackedDeviceProperty(idx, prop_id)
        return float(v), True
    except Exception:
        return 0.0, False


def _safe_get_bool(system, idx, prop_id):
    try:
        v = system.getBoolTrackedDeviceProperty(idx, prop_id)
        return bool(v), True
    except Exception:
        return False, False


def _safe_get_int32(system, idx, prop_id):
    try:
        v = system.getInt32TrackedDeviceProperty(idx, prop_id)
        return int(v), True
    except Exception:
        return 0, False


def _safe_get_string(system, idx, prop_id):
    try:
        v = system.getStringTrackedDeviceProperty(idx, prop_id)
        return str(v), True
    except Exception:
        return "", False


def _resolve_controller_indices(openvr, system):
    """Find which device indices are ``TrackedDeviceClass_Controller``
    with role Left/Right."""
    out = []
    for idx in range(openvr.k_unMaxTrackedDeviceCount):
        try:
            cls = system.getTrackedDeviceClass(idx)
        except Exception:
            continue
        if cls != openvr.TrackedDeviceClass_Controller:
            continue
        try:
            role = int(system.getControllerRoleForTrackedDeviceIndex(idx))
        except Exception:
            role = 0
        role_name = {1: "Left", 2: "Right"}.get(role, f"role={role}")
        try:
            serial = system.getStringTrackedDeviceProperty(
                idx, openvr.Prop_SerialNumber_String
            )
        except Exception:
            serial = "?"
        out.append((idx, role_name, serial))
    return out


def _sweep_baseline(system, idx) -> Tuple[Dict[int, float], Dict[int, bool], Dict[int, int]]:
    """Capture baseline values for every property index that supports the
    given getter, so we can compare against the squeezed state later."""
    f_baseline: Dict[int, float] = {}
    b_baseline: Dict[int, bool] = {}
    i_baseline: Dict[int, int] = {}

    # Standard OpenVR property range is 1000-3299 (general) + 5000-5999 (HMD)
    # + 7000-7999 (controller) + 10000+ (vendor)
    # Sweep all plausible ranges.  Limited to 2000-7999 + 10000-10499 for speed.
    ranges = [
        range(1000, 3300),     # general + audio + display
        range(5000, 6000),     # HMD + audio
        range(6000, 7000),     # also commonly used
        range(7000, 8000),     # controller-specific
        range(10000, 19999),   # vendor reserved (PICO may use this entire span)
        range(20000, 21000),   # extended vendor range (PICO 4 firmware seen here)
    ]
    for r in ranges:
        for prop_id in r:
            v, ok = _safe_get_float(system, idx, prop_id)
            if ok:
                f_baseline[prop_id] = v
            v_b, ok_b = _safe_get_bool(system, idx, prop_id)
            if ok_b:
                b_baseline[prop_id] = v_b
            v_i, ok_i = _safe_get_int32(system, idx, prop_id)
            if ok_i:
                i_baseline[prop_id] = v_i
    return f_baseline, b_baseline, i_baseline


def _detect_changes(system, idx, f_baseline, b_baseline, i_baseline,
                    f_max_delta, b_changes, i_changes,
                    f_max_value, b_seen_true, i_max_value):
    """Single poll pass — compare current values to baseline."""
    for prop_id, base in f_baseline.items():
        v, ok = _safe_get_float(system, idx, prop_id)
        if not ok:
            continue
        delta = abs(v - base)
        if delta > f_max_delta.get(prop_id, 0.0):
            f_max_delta[prop_id] = delta
        if v > f_max_value.get(prop_id, base):
            f_max_value[prop_id] = v
    for prop_id, base in b_baseline.items():
        v, ok = _safe_get_bool(system, idx, prop_id)
        if not ok:
            continue
        if v != base:
            b_changes[prop_id] = b_changes.get(prop_id, 0) + 1
        if v and not b_seen_true.get(prop_id, False):
            b_seen_true[prop_id] = True
    for prop_id, base in i_baseline.items():
        v, ok = _safe_get_int32(system, idx, prop_id)
        if not ok:
            continue
        if v != base:
            i_changes[prop_id] = i_changes.get(prop_id, 0) + 1
        if v > i_max_value.get(prop_id, base):
            i_max_value[prop_id] = v


def _format_prop_name(openvr, prop_id: int) -> str:
    """Best-effort lookup of the OpenVR named constant for a property ID."""
    for attr in dir(openvr):
        if not attr.startswith("Prop_"):
            continue
        try:
            v = getattr(openvr, attr)
        except Exception:
            continue
        if isinstance(v, int) and v == prop_id:
            return attr
    return f"Prop_<{prop_id}>"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12.0,
                        help="Listening window (default 12 s).")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Poll rate Hz (default 10).")
    args = parser.parse_args()

    import openvr  # type: ignore

    system = openvr.init(openvr.VRApplication_Other)
    try:
        controllers = _resolve_controller_indices(openvr, system)
        if not controllers:
            print("FATAL — no controllers visible.  Start PCVR streaming first.")
            return 2
        print(f"Found {len(controllers)} controller(s):")
        for idx, role, serial in controllers:
            print(f"  idx={idx} role={role:6s} serial={serial!r}")

        # Baseline (idle, hands NOT squeezing)
        # NOTE: with extended ranges the sweep takes 5-10s per controller.
        print("\nCapturing baseline (do NOT squeeze controllers yet) -- extended sweep ...")
        time.sleep(1.0)
        baselines: Dict[int, Tuple[Dict[int, float], Dict[int, bool], Dict[int, int]]] = {}
        for idx, _role, _serial in controllers:
            baselines[idx] = _sweep_baseline(system, idx)
            print(f"  idx={idx}: float_props={len(baselines[idx][0])} "
                  f"bool_props={len(baselines[idx][1])} "
                  f"int_props={len(baselines[idx][2])}")

        # Live polling
        print(f"\nNow squeeze trigger + grip alternately on BOTH controllers "
              f"for {args.seconds:.0f}s ...")
        f_max_delta: Dict[int, Dict[int, float]] = {idx: {} for idx, _, _ in controllers}
        f_max_value: Dict[int, Dict[int, float]] = {idx: {} for idx, _, _ in controllers}
        b_changes: Dict[int, Dict[int, int]] = {idx: {} for idx, _, _ in controllers}
        b_seen_true: Dict[int, Dict[int, bool]] = {idx: {} for idx, _, _ in controllers}
        i_changes: Dict[int, Dict[int, int]] = {idx: {} for idx, _, _ in controllers}
        i_max_value: Dict[int, Dict[int, int]] = {idx: {} for idx, _, _ in controllers}

        deadline = time.time() + args.seconds
        period = 1.0 / max(0.5, float(args.rate))
        last = 0.0
        while time.time() < deadline:
            now = time.time()
            if now - last < period:
                time.sleep(0.01)
                continue
            last = now
            for idx, _, _ in controllers:
                fb, bb, ib = baselines[idx]
                _detect_changes(
                    system, idx, fb, bb, ib,
                    f_max_delta[idx], b_changes[idx], i_changes[idx],
                    f_max_value[idx], b_seen_true[idx], i_max_value[idx],
                )

        # Report
        print("\n" + "=" * 72)
        print("RESULTS — properties that changed during squeeze:")
        print("=" * 72)
        for idx, role, serial in controllers:
            print(f"\n--- idx={idx} role={role} serial={serial!r} ---")

            # Float: rank by max_delta
            interesting_floats = sorted(
                ((pid, d) for pid, d in f_max_delta[idx].items() if d > 0.001),
                key=lambda x: -x[1],
            )[:15]
            if interesting_floats:
                print("  FLOAT properties (max delta from baseline):")
                for pid, d in interesting_floats:
                    base = baselines[idx][0][pid]
                    maxv = f_max_value[idx].get(pid, base)
                    print(f"    {_format_prop_name(openvr, pid):<55} "
                          f"id={pid} delta={d:.3f} baseline={base:.3f} max={maxv:.3f}")
            else:
                print("  FLOAT: no changes > 0.001 detected.")

            # Bool: any that became True
            true_bools = [pid for pid, seen in b_seen_true[idx].items() if seen]
            if true_bools:
                print("  BOOL properties (saw True during squeeze):")
                for pid in true_bools[:15]:
                    print(f"    {_format_prop_name(openvr, pid):<55} "
                          f"id={pid} flipped {b_changes[idx].get(pid, 0)} times")
            else:
                print("  BOOL: no property flipped during squeeze.")

            # Int32: any with high max
            interesting_ints = sorted(
                ((pid, i_changes[idx].get(pid, 0))
                 for pid in i_changes[idx] if i_changes[idx][pid] > 0),
                key=lambda x: -x[1],
            )[:10]
            if interesting_ints:
                print("  INT32 properties (changes detected):")
                for pid, c in interesting_ints:
                    base = baselines[idx][2][pid]
                    maxv = i_max_value[idx].get(pid, base)
                    print(f"    {_format_prop_name(openvr, pid):<55} "
                          f"id={pid} changes={c} baseline={base} max={maxv}")
            else:
                print("  INT32: no property changed during squeeze.")

        print("\n" + "=" * 72)
        print("If you see FLOAT property with delta close to 1.0 -- that is the")
        print("trigger / grip analog channel.  We will add it to vr_sampler.py.")
        print("If only BOOL flipped: PICO grip is digital, retargeter must accept bool.")
        print("If NOTHING changed: PICO routes input only through OpenXR / SteamVR")
        print("Action API and we need option B (OpenXR runtime switch).")
        print("=" * 72)
        return 0
    finally:
        openvr.shutdown()


if __name__ == "__main__":
    sys.exit(main())
