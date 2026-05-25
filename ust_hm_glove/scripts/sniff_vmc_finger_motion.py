"""Live VMC bone-motion sniffer.

Listens on UDP 39539 for ``/VMC/Ext/Bone/Pos`` packets, tracks per-bone
quaternion ranges (max - min for each of qx/qy/qz/qw), and reports which
bones show meaningful variation during the capture window.

Run this WHILE you actively move your fingers (open / fist / pinch /
spread).  If a bone's quaternion components stay nearly constant across
the capture, that bone is not responding to finger motion — UDCAP is
either streaming a static T-pose for that bone or the calibration is
locking it.

Usage::

    python -X utf8 -m ust_ws.ust_hm_glove.scripts.sniff_vmc_finger_motion [--seconds 8]

Key questions this answers:

  * Does ``LeftIndexProximal`` move when you curl your left index?
    (Variation in qx/qy/qz components > ~0.01)
  * Does ``LeftThumbProximal`` move on opposition?
  * Are any FINGER bones static (rotation never changes)?  That would
    explain "retargeter sees nonzero values but robot fingers don't
    move with my hand" -- UDCAP would be emitting a constant pose
    rather than tracked sensor data.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from typing import Dict, List, Tuple


def _parse_osc_string(buf: bytes, pos: int) -> Tuple[str, int]:
    end = buf.index(b"\x00", pos)
    s = buf[pos:end].decode("ascii", errors="replace")
    pad = ((end // 4) + 1) * 4
    return s, pad


def _parse_floats(buf: bytes, pos: int, n: int) -> Tuple[List[float], int]:
    out: List[float] = []
    for _ in range(n):
        if pos + 4 > len(buf):
            break
        out.append(struct.unpack(">f", buf[pos:pos + 4])[0])
        pos += 4
    return out, pos


def parse_vmc_bone_pos(packet: bytes):
    """Return (bone_name, (px, py, pz, qx, qy, qz, qw)) or None."""
    try:
        addr, p = _parse_osc_string(packet, 0)
        if addr != "/VMC/Ext/Bone/Pos":
            return None
        type_tag, p = _parse_osc_string(packet, p)
        # Expected: ',sfffffff' (string + 7 floats)
        if not type_tag.startswith(",s") or type_tag.count("f") < 7:
            return None
        name, p = _parse_osc_string(packet, p)
        floats, _ = _parse_floats(packet, p, 7)
        if len(floats) < 7:
            return None
        return name, tuple(floats)
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=39539, help="VMC UDP port (default: 39539)")
    ap.add_argument("--seconds", type=float, default=8.0, help="Capture window in seconds")
    ap.add_argument("--threshold", type=float, default=0.02,
                    help="Quat-component range below this is considered 'static'")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError as exc:
        print(f"[ERROR] Could not bind UDP {args.port}: {exc}")
        print("  Another process may already be listening (the teleop app, perhaps).")
        return 1
    sock.settimeout(0.25)

    print(f"Sniffing VMC on UDP {args.port} for {args.seconds:.0f} seconds.")
    print()
    print("  >>> MOVE YOUR FINGERS NOW.  Open / fist / open / fist, both hands. <<<")
    print()
    print("Live readout below shows the LeftIndexProximal quaternion every 0.5 s.")
    print("If the values change as you curl/release the LEFT INDEX finger, UDCAP")
    print("IS streaming live data and the diagnostic verdict will be 'live'.  If")
    print("they stay constant despite you moving, UDCAP is broadcasting a frozen")
    print("snapshot.")
    print()

    bone_stats: Dict[str, Dict[str, Tuple[float, float]]] = {}
    bone_counts: Dict[str, int] = {}
    deadline = time.perf_counter() + args.seconds
    next_readout = time.perf_counter() + 0.5
    last_seen: Dict[str, Tuple[float, float, float, float]] = {}
    while time.perf_counter() < deadline:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            pass
        else:
            parsed = parse_vmc_bone_pos(data)
            if parsed is not None:
                name, vals = parsed
                _, _, _, qx, qy, qz, qw = vals
                bone_counts[name] = bone_counts.get(name, 0) + 1
                last_seen[name] = (qx, qy, qz, qw)
                if name not in bone_stats:
                    bone_stats[name] = {
                        "qx": (qx, qx),
                        "qy": (qy, qy),
                        "qz": (qz, qz),
                        "qw": (qw, qw),
                    }
                else:
                    for key, val in (("qx", qx), ("qy", qy), ("qz", qz), ("qw", qw)):
                        lo, hi = bone_stats[name][key]
                        bone_stats[name][key] = (min(lo, val), max(hi, val))
        # Periodic live readout so the user gets instant feedback while moving.
        now = time.perf_counter()
        if now >= next_readout:
            next_readout = now + 0.5
            remain = max(0.0, deadline - now)
            probe = last_seen.get("LeftIndexProximal")
            if probe is not None:
                qx, qy, qz, qw = probe
                print(
                    f"  [t={args.seconds - remain:5.1f}s]  "
                    f"LeftIndexProximal quat=({qx:+.3f},{qy:+.3f},{qz:+.3f},{qw:+.3f})"
                    f"   (curl/release LEFT INDEX now)"
                )
            else:
                print(f"  [t={args.seconds - remain:5.1f}s]  (no LeftIndexProximal packet yet)")
    sock.close()

    finger_keywords = ("Thumb", "Index", "Middle", "Ring", "Little")
    finger_bones = sorted(n for n in bone_stats if any(k in n for k in finger_keywords))
    body_bones = sorted(n for n in bone_stats if n not in finger_bones)

    def _row(name: str) -> str:
        s = bone_stats[name]
        ranges = {k: hi - lo for k, (lo, hi) in s.items()}
        max_range = max(ranges.values())
        flag = "static" if max_range < args.threshold else "  live"
        return (
            f"  [{flag}] {name:30s} count={bone_counts[name]:5d} "
            f"qx_rng={ranges['qx']:.3f} qy_rng={ranges['qy']:.3f} "
            f"qz_rng={ranges['qz']:.3f} qw_rng={ranges['qw']:.3f}"
        )

    print()
    print(f"{'='*78}")
    print(f"VMC bone variation report  (threshold={args.threshold:.3f}, window={args.seconds:.0f}s)")
    print(f"{'='*78}")
    print()
    print(f"FINGER BONES ({len(finger_bones)}):")
    print()
    for name in finger_bones:
        print(_row(name))
    print()
    print(f"BODY BONES ({len(body_bones)}):")
    print()
    for name in body_bones:
        print(_row(name))

    static_fingers = [n for n in finger_bones
                      if max((hi - lo) for _, (lo, hi) in bone_stats[n].items()) < args.threshold]
    live_fingers = [n for n in finger_bones if n not in static_fingers]

    print()
    print(f"{'-'*78}")
    print(f"VERDICT")
    print(f"{'-'*78}")
    print(f"  finger bones live   : {len(live_fingers)}/{len(finger_bones)}")
    print(f"  finger bones static : {len(static_fingers)}/{len(finger_bones)}")
    if not live_fingers:
        print()
        print("  * UDCAP is broadcasting finger bones but NONE of them change values")
        print("    during the capture window.  Most likely causes:")
        print("    1. The capture happened while you weren't moving your fingers --")
        print("       re-run and CONTINUOUSLY squeeze/release through the whole window.")
        print("    2. UDCAP is in a 'pose passthrough' mode where it streams the")
        print("       calibration T-pose for fingers regardless of glove sensor input.")
        print("       Check UDCAP UI > Setting:")
        print("       - 'Trigger' / 'Grip' / 'TrackPad' toggles must be ON for the")
        print("         per-finger output to mirror the sensor (per the screenshot,")
        print("         these were enabled with thresholds set, but there's also a")
        print("         'Quick Disable' switch that may be ON).")
        print("       - 'Controller Priority: High' in General can also suppress")
        print("         finger output when an underlying physical controller is")
        print("         tracked alongside the gloves.  Try 'Low'.")
        print("    3. The gloves were calibrated incorrectly -- repeat F1 calibration")
        print("       carefully, holding each pose for ~2 seconds.")
    elif len(live_fingers) < len(finger_bones) * 0.5:
        print()
        print("  * Only some finger bones are live.  Check whether the static ones")
        print("    correspond to sensors UDCAP can't read (spread / abduction sensors")
        print("    are often missing on entry-level gloves).")
    else:
        print()
        print("  * Finger bones ARE live.  If the robot still doesn't move its")
        print("    fingers, the issue is downstream -- in the retargeter mapping or")
        print("    the Pink IK action term.  Inspect the actual action[14:36] values")
        print("    during teleop (e.g., add a per-frame print in the device's")
        print("    advance() loop or watch the [GR1T2Retarget #N] log lines).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
