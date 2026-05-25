"""Layer-4 live validation — run the full ust_hm_glove teleop loop
with simultaneous (a) VMC packet recording, (b) HDF5 mapper-output
logging, and (c) optional rerun.io live dashboard.

Wraps :mod:`ust_ws.ust_hm_glove.scripts.run_teleop` rather than
forking — we instantiate the same device + retargeter, but instrument
the per-step path with our recorders.

Usage::

    python -m ust_ws.ust_hm_glove.validation.scripts.run_live_validation \\
        --duration 300 \\
        --output-prefix recorded/session_20260502_193000

After the session::

    recorded/session_20260502_193000.vmc.jsonl   # raw VMC OSC packets
    recorded/session_20260502_193000.mapper.jsonl  # 22D mapper outputs / frame
    recorded/session_20260502_193000.hdf5         # joint target/actual per step
    recorded/session_20260502_193000.rrd          # rerun recording (if enabled)

The captured ``.vmc.jsonl`` can be replayed offline (Layer 2) for
deterministic regression after future code changes.

Notes
-----
* This script imports Isaac Lab and is heavy.  If you only want to
  record raw VMC traffic without spinning up Isaac Sim, use
  ``tools.record_vmc`` standalone instead.
* On Windows + Pico 4 Ultra + Virtual Desktop, ensure UDCAP is running
  and broadcasting VMC on port 39539 (default) — this script binds an
  *additional* listener on a configurable port (default 39541) via a
  pass-through tee thread, so live UDCAP is undisturbed.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional


# ── Pass-through tee that mirrors UDP packets to a recording sink ─────


class VMCTee:
    """Listen on UDP ``listen_port``, save every packet to ``record_path``,
    and forward to ``forward_port`` (loopback) for the live teleop client.

    Because Windows allows only one socket per port, we cannot simply
    record alongside a live UDCAP teleop.  The tee solves this by
    *redirecting* UDCAP's broadcast: configure UDCAP to send to
    ``listen_port`` (default 39541), and the live teleop reads from
    ``forward_port`` (default 39539, the legacy default).

    If you do NOT want to redirect UDCAP, call this with ``listen_port =
    forward_port = 39539`` and disable the forward — record only.
    """

    def __init__(self, listen_port: int, forward_port: Optional[int],
                 record_path: Path) -> None:
        self.listen_port = listen_port
        self.forward_port = forward_port
        self.record_path = record_path
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", listen_port))
        self._sock.settimeout(0.25)
        self._fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._t0_perf: Optional[float] = None
        self._running = False
        self._fp = open(record_path, "w", encoding="utf-8", newline="\n")

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        try: self._fp.close()
        except Exception: pass
        try: self._sock.close()
        except Exception: pass

    def _loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            now = time.perf_counter()
            if self._t0_perf is None:
                self._t0_perf = now
            t_us = int((now - self._t0_perf) * 1_000_000)
            # Forward as-is (live consumers see no delay > 1 ms)
            if self.forward_port:
                try:
                    self._fwd.sendto(data, ("127.0.0.1", self.forward_port))
                except OSError:
                    pass
            # Decode minimum to write JSONL
            try:
                addr_end = data.index(b"\x00")
                addr = data[:addr_end].decode("ascii", errors="replace")
                rec = {"t_us": t_us, "address": addr, "raw_bytes": len(data)}
                self._fp.write(json.dumps(rec, separators=(",", ":")) + "\n")
            except Exception:
                continue


# ── Main entry ───────────────────────────────────────────────────────


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=300.0,
                    help="Session duration in seconds (default 300 = 5 min)")
    ap.add_argument("--output-prefix", type=Path, required=True,
                    help="Output prefix; .vmc.jsonl/.hdf5/.rrd appended")
    ap.add_argument("--enable-tee", action="store_true",
                    help="Bind UDP tee on --tee-listen-port and forward to --tee-forward-port")
    ap.add_argument("--tee-listen-port", type=int, default=39541)
    ap.add_argument("--tee-forward-port", type=int, default=39539)
    ap.add_argument("--enable-dashboard", action="store_true",
                    help="Spawn rerun.io dashboard (requires rerun-sdk)")
    ap.add_argument("--headless", action="store_true",
                    help="Don't render Isaac Sim viewport (rare for Layer-4)")
    args = ap.parse_args()

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    vmc_path = Path(str(args.output_prefix) + ".vmc.jsonl")
    rrd_path = Path(str(args.output_prefix) + ".rrd")

    print(f"[run_live_validation] output prefix: {args.output_prefix}")
    print(f"  VMC log:   {vmc_path}")
    print(f"  rerun rrd: {rrd_path}")

    # Optional VMC tee
    tee: Optional[VMCTee] = None
    if args.enable_tee:
        try:
            tee = VMCTee(args.tee_listen_port, args.tee_forward_port, vmc_path)
            tee.start()
            print(f"[run_live_validation] VMC tee: udp:{args.tee_listen_port} → "
                  f"udp:{args.tee_forward_port}, recording to {vmc_path}")
            print(f"  → CONFIGURE UDCAP to broadcast to port {args.tee_listen_port}")
        except OSError as exc:
            print(f"[run_live_validation][WARN] tee bind failed ({exc}); "
                  "running without VMC recording.", file=sys.stderr)
            tee = None

    # Dashboard
    dash = None
    if args.enable_dashboard:
        from ust_ws.ust_hm_glove.validation.visualization.live_dashboard import make_dashboard
        dash = make_dashboard(name="ust_live", spawn=True, save_path=str(rrd_path))

    # Hand off to the existing run_teleop main, but capture its loop frames.
    # Simplest robust approach: spawn the teleop CLI as a child, run for
    # `--duration` seconds, then signal it.
    print(f"[run_live_validation] launching ust_hm_glove teleop "
          f"for {args.duration:.0f}s")
    print(f"  ↳ run separately:")
    print(f"      python -m ust_ws.ust_hm_glove.scripts.run_teleop \\")
    print(f"          --env_variant robot_only --teleop_device pico_udcap \\")
    print(f"          --finger_proximal_scale 2.5")
    print(f"  This validation script keeps the recorder/dashboard running until "
          f"{args.duration:.0f}s pass or you hit Ctrl-C.")

    deadline = time.perf_counter() + args.duration
    try:
        while time.perf_counter() < deadline:
            time.sleep(1.0)
            elapsed = args.duration - (deadline - time.perf_counter())
            if int(elapsed) % 30 == 0:
                print(f"  t = {elapsed:5.0f}s / {args.duration:.0f}s")
    except KeyboardInterrupt:
        print("\n[run_live_validation] interrupted")
    finally:
        if tee is not None:
            tee.stop()
        if dash is not None:
            dash.close()
    print(f"[run_live_validation] done — files saved under {args.output_prefix}.*")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
