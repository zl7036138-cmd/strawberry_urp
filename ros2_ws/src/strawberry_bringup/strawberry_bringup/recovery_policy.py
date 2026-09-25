"""Fail-closed interpretation of manipulation recovery dispositions."""

from __future__ import annotations

from enum import IntEnum, StrEnum


class RecoveryDisposition(IntEnum):
    """Wire values shared with ``PickAndPlace.action``.

    The zero value is deliberately the most restrictive so an absent,
    malformed, or future value cannot authorize motion accidentally.
    """

    MOTION_WITHHELD = 0
    AT_HOME = 1
    HOME_REQUIRED = 2


class RecoveryStep(StrEnum):
    WITHHOLD = "WITHHOLD"
    ALREADY_HOME = "ALREADY_HOME"
    REQUEST_HOME = "REQUEST_HOME"


def recovery_step_for_disposition(value: int) -> RecoveryStep:
    """Map a wire value to the only permitted orchestrator recovery step."""

    try:
        disposition = RecoveryDisposition(int(value))
    except (TypeError, ValueError, OverflowError):
        return RecoveryStep.WITHHOLD
    if disposition is RecoveryDisposition.AT_HOME:
        return RecoveryStep.ALREADY_HOME
    if disposition is RecoveryDisposition.HOME_REQUIRED:
        return RecoveryStep.REQUEST_HOME
    return RecoveryStep.WITHHOLD
