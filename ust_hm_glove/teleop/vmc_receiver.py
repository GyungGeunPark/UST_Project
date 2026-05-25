"""VMC Broadcast OSC receiver (Path B fallback for UDCAP VR Glove).

Reference: `research/28 …` §2.4.  The UDCAP Driver can broadcast its hand
bones as Unity ``HumanBodyBones`` names over UDP using the VMC protocol.
If the SteamVR Skeletal Input path fails (driver not loaded, session
conflict, …), we fall back to decoding VMC messages here.

Only the hand bones are consumed — waist/ankle data will continue to come
from the Pico Enhanced Forearm trackers via SteamVR.

Usage
-----

    receiver = VMCHandReceiver(port=39539)
    receiver.start()
    ...
    bones = receiver.latest_bones()   # dict[name] -> (qx, qy, qz, qw)
    receiver.stop()

Each bone value is stored as a 4-tuple of floats in ``(qx, qy, qz, qw)``
order to match ``UDCAPFingerMapper.map_hand``.
"""

from __future__ import annotations

import threading
from typing import Dict, Optional, Tuple


__all__ = ["VMCHandReceiver", "HAND_BONE_NAMES"]


# Accept both naming conventions for the thumb (Unity HumanBodyBones vs
# anatomy).  UDCAP 0.1.8.2 uses Unity convention (Proximal/Intermediate/
# Distal for every finger including the thumb); other VMC broadcasters
# use the anatomy convention (Metacarpal/Proximal/Distal for the thumb).
# Storing all of them is harmless — the mapper picks whichever subset is
# present per-frame.  Without this, UDCAP's ThumbIntermediate would be
# silently filtered out and downstream thumb pitch / opposition would
# stay at 0.  See memory.md §10.16.
HAND_BONE_NAMES = tuple(
    f"{side}{finger}{segment}"
    for side in ("Left", "Right")
    for finger in ("Thumb", "Index", "Middle", "Ring", "Little")
    for segment in (
        ("Metacarpal", "Proximal", "Intermediate", "Distal") if finger == "Thumb"
        else ("Proximal", "Intermediate", "Distal")
    )
)


class VMCHandReceiver:
    """Threaded UDP listener for VMC ``/VMC/Ext/Bone/Pos`` messages."""

    def __init__(self, port: int = 39539, host: str = "0.0.0.0", bone_filter=None):
        self.port = int(port)
        self.host = host
        self._bone_filter = set(bone_filter) if bone_filter else set(HAND_BONE_NAMES)
        self._lock = threading.Lock()
        self._bones: Dict[str, Tuple[float, float, float, float]] = {}
        self._server = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._message_count = 0

    # ── lifecycle ──────────────────────────────────────────────────────────

    def start(self) -> None:
        from pythonosc import dispatcher, osc_server  # noqa: WPS433

        d = dispatcher.Dispatcher()
        d.map("/VMC/Ext/Bone/Pos", self._on_bone)
        self._server = osc_server.ThreadingOSCUDPServer((self.host, self.port), d)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name=f"VMCHandReceiver:{self.port}",
            daemon=True,
        )
        self._thread.start()
        self._running = True

    def stop(self) -> None:
        if self._server is not None:
            try:
                self._server.shutdown()
            except Exception:
                pass
            self._server = None
        self._thread = None
        self._running = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    # ── access ─────────────────────────────────────────────────────────────

    def latest_bones(self) -> Dict[str, Tuple[float, float, float, float]]:
        """Return a shallow copy of the most recent bone -> (qx, qy, qz, qw) dict."""
        with self._lock:
            return dict(self._bones)

    @property
    def message_count(self) -> int:
        return self._message_count

    @property
    def is_running(self) -> bool:
        return self._running

    # ── OSC callback ──────────────────────────────────────────────────────

    def _on_bone(self, _addr, *args) -> None:
        # VMC spec: (name, px, py, pz, qx, qy, qz, qw)
        if len(args) < 8:
            return
        name = str(args[0])
        self._message_count += 1
        if name not in self._bone_filter:
            return
        try:
            qx, qy, qz, qw = float(args[4]), float(args[5]), float(args[6]), float(args[7])
        except (TypeError, ValueError):
            return
        with self._lock:
            self._bones[name] = (qx, qy, qz, qw)
