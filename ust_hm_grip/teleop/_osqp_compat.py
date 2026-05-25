"""Compatibility shim for ``osqp`` 0.6 ↔ ``qpsolvers`` >=4.x.

Problem
-------
The ``ust`` conda env pins ``osqp==0.6.7.post3`` (hard dependency of
``isaacsim-core==5.1.0.0``).  Our ``qpsolvers==4.11.0`` ships an osqp
adapter that does::

    from osqp import OSQP, SolverStatus  # line 25 of qpsolvers/solvers/osqp_.py
    ...
    solution.found = res.info.status_val == SolverStatus.OSQP_SOLVED

``SolverStatus`` was introduced in **osqp 1.0**.  osqp 0.6 only exposes the
integer status codes directly (``status_val == 1`` means solved).  Without
this shim the import raises ``ImportError`` and ``qpsolvers`` reports
``available_solvers == []``, so Isaac Lab's hardcoded ``solver="osqp"`` call
in ``pink_ik.py:224`` fails every frame — Pink IK returns the current joint
positions unchanged and the robot stands still.

Upgrading osqp to >=1.0 is NOT an option because it breaks the
``isaacsim-core==0.6.7.post3`` pin.  Downgrading ``qpsolvers`` would
require pinning ``pin-pink`` to an older release (cascade of version
conflicts).

Fix
---
Inject a ``SolverStatus`` ``IntEnum`` into the ``osqp`` module namespace
**before** ``qpsolvers`` imports it.  Because osqp 0.6's ``status_val``
integer codes are the same as the ones osqp 1.0's enum wraps, the
``status_val == SolverStatus.OSQP_SOLVED`` comparison in qpsolvers still
yields the correct answer (both sides are ``int(1)``).

Calling :func:`apply` is idempotent and cheap — safe to call multiple
times from any module-level import site.
"""

from __future__ import annotations

import enum


# Integer codes from osqp's C enum ``osqp_status_type``.  Source:
# https://github.com/osqp/osqp/blob/v0.6.2/include/osqp.h#L119
class _SolverStatusCompat(enum.IntEnum):
    """Mirror of osqp >=1.0's ``SolverStatus`` enum using osqp 0.6 values."""

    OSQP_DUAL_INFEASIBLE_INACCURATE = 4
    OSQP_PRIMAL_INFEASIBLE_INACCURATE = 3
    OSQP_SOLVED_INACCURATE = 2
    OSQP_SOLVED = 1
    OSQP_MAX_ITER_REACHED = -2
    OSQP_PRIMAL_INFEASIBLE = -3
    OSQP_DUAL_INFEASIBLE = -4
    OSQP_SIGINT = -5
    OSQP_TIME_LIMIT_REACHED = -6
    OSQP_NON_CVX = -7
    OSQP_UNSOLVED = -10


_applied = False


def apply() -> bool:
    """Inject :class:`_SolverStatusCompat` into ``osqp.SolverStatus``.

    Returns
    -------
    ``True`` on success, ``False`` if osqp itself failed to import (which
    means Pink IK wouldn't work anyway — caller should log + surrender).
    """
    global _applied
    if _applied:
        return True
    try:
        import osqp
    except Exception:
        return False

    # Respect osqp >=1.0 if someone has already installed it.
    if hasattr(osqp, "SolverStatus"):
        _applied = True
        return True

    osqp.SolverStatus = _SolverStatusCompat  # type: ignore[attr-defined]
    _applied = True
    return True


# Apply on import so any ``from ust_ws.ust_hm_glove.teleop._osqp_compat
# import apply`` call site also triggers the patch even if ``apply()`` is
# never explicitly called.
apply()
