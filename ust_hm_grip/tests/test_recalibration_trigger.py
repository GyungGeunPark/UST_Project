"""Rising-edge + cooldown logic for the A-button recalibration trigger.

The actual trigger lives inside ``run_teleop.py``'s ``_phase_d_apply``
closure, which is hard to unit-test in isolation.  This test mirrors
the trigger's state-machine in a small standalone helper so we can
verify the rising-edge + cooldown semantics that the closure relies on.
If the behaviour ever diverges from the closure, update both at once.
"""
from __future__ import annotations
import pytest


_COOLDOWN_SEC = 0.5


def _make_trigger():
    """Factory for a fresh trigger state.  Mirrors ``_fb`` fields used
    by ``_phase_d_check_recalibration`` in run_teleop.py."""
    state = {"prev": False, "last_fire": -1.0}

    def step(a_now: bool, now: float) -> bool:
        a_prev = state["prev"]
        state["prev"] = a_now
        if a_now and not a_prev:
            if (now - state["last_fire"]) > _COOLDOWN_SEC:
                state["last_fire"] = now
                return True
        return False

    return step


def test_rising_edge_fires_once():
    """A held-down button must fire exactly once."""
    fire = _make_trigger()
    assert fire(True, 0.0) is True   # rising edge
    assert fire(True, 0.05) is False  # still held → no fire
    assert fire(True, 0.10) is False
    assert fire(True, 0.5) is False
    assert fire(True, 1.0) is False


def test_release_then_press_fires_again_after_cooldown():
    fire = _make_trigger()
    assert fire(True, 0.0) is True
    assert fire(False, 0.1) is False  # released, no fire
    # Press again immediately — within cooldown, should NOT fire
    assert fire(True, 0.2) is False
    assert fire(False, 0.3) is False
    # Press again after cooldown → fires
    assert fire(True, 0.7) is True


def test_no_fire_without_press():
    fire = _make_trigger()
    for t in range(10):
        assert fire(False, t * 0.1) is False


def test_first_ever_press_fires_even_before_cooldown_elapsed():
    """The initial last_fire = -1.0 (sentinel) guarantees the first
    press always fires, even at t=0."""
    fire = _make_trigger()
    assert fire(True, 0.0) is True


def test_multiple_users_independent_state():
    """Two independent triggers should not interfere with each other."""
    fire_a = _make_trigger()
    fire_b = _make_trigger()
    assert fire_a(True, 0.0) is True
    assert fire_b(True, 0.0) is True   # b's first press, independent of a
    assert fire_a(False, 0.1) is False
    assert fire_a(True, 0.6) is True   # a fires again after cooldown
    # b should be unaffected
    assert fire_b(True, 0.1) is False  # b still in cooldown


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
