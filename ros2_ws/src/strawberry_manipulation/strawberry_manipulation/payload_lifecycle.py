"""Fail-closed lifecycle vocabulary for a grasped fruit payload.

The action definition deliberately remains stable.  This module is the pure
state boundary shared by the executor, unit tests, future recorder adapters and
the planned phase-aware collision policy.
"""

from __future__ import annotations

from enum import IntEnum


class PayloadState(IntEnum):
    """Physical and planning-scene ownership state of one selected fruit."""

    EMPTY = 0
    CONTACT = 1
    HOLDING = 2
    ESCAPED = 3
    AT_BIN = 4
    RELEASED = 5

    @property
    def retains_fruit(self) -> bool:
        """Whether attachment and carried-body geometry must remain installed."""

        return self in (self.HOLDING, self.ESCAPED, self.AT_BIN)
