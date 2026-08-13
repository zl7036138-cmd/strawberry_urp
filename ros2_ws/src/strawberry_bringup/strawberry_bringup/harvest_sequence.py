"""Dependency-free continuous harvest batch state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class HarvestState(str, Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    OBSERVING = "OBSERVING"
    CONFIRMING = "CONFIRMING"
    PICKING = "PICKING"
    DONE = "DONE"


@dataclass(frozen=True)
class HarvestEvent:
    state: HarvestState
    target_id: int | None
    outcome: str
    detail: str = ""


class HarvestSequence:
    def __init__(self, *, max_attempts_per_target: int = 2, max_targets: int = 12) -> None:
        if max_attempts_per_target != 2:
            raise ValueError("continuous harvesting permits exactly one retry")
        if max_targets <= 0:
            raise ValueError("max_targets must be positive")
        self.max_attempts_per_target = max_attempts_per_target
        self.max_targets = max_targets
        self.state = HarvestState.IDLE
        self.current_target_id: int | None = None
        self.retry_target_id: int | None = None
        self.attempts: dict[int, int] = {}
        self.harvested: list[int] = []
        self.skipped: dict[int, str] = {}
        self.failures: list[dict[str, Any]] = []
        self.history: list[HarvestEvent] = []

    @property
    def terminal(self) -> bool:
        return self.state is HarvestState.DONE

    @property
    def completed_ids(self) -> frozenset[int]:
        return frozenset(self.harvested) | frozenset(self.skipped)

    def start(self) -> None:
        if self.state not in {HarvestState.IDLE, HarvestState.DONE}:
            raise ValueError("harvest sequence is already active")
        self.__init__(max_attempts_per_target=self.max_attempts_per_target, max_targets=self.max_targets)
        self.state = HarvestState.SCANNING
        self.history.append(HarvestEvent(self.state, None, "STARTED"))

    def select(self, target_id: int) -> None:
        if self.state is not HarvestState.SCANNING:
            raise ValueError("target selection is valid only while scanning")
        if target_id <= 0 or target_id in self.completed_ids:
            raise ValueError("selected target is invalid or already complete")
        if self.retry_target_id is not None and target_id != self.retry_target_id:
            raise ValueError("a retry is reserved for another target")
        self.current_target_id = target_id
        self.attempts[target_id] = self.attempts.get(target_id, 0) + 1
        self.state = HarvestState.OBSERVING
        self.history.append(HarvestEvent(self.state, target_id, "SELECTED"))

    def observation_result(self, success: bool, detail: str = "") -> bool:
        if self.state is not HarvestState.OBSERVING or self.current_target_id is None:
            raise ValueError("observation result has no active target")
        if success:
            self.state = HarvestState.CONFIRMING
            self.history.append(HarvestEvent(self.state, self.current_target_id, "OBSERVED", detail))
            return True
        self._attempt_failed("OBSERVATION_FAILED", detail)
        return False

    def confirmation_result(self, success: bool, detail: str = "") -> bool:
        if self.state is not HarvestState.CONFIRMING or self.current_target_id is None:
            raise ValueError("confirmation result has no active target")
        if success:
            self.state = HarvestState.PICKING
            self.history.append(HarvestEvent(self.state, self.current_target_id, "CONFIRMED", detail))
            return True
        self._attempt_failed("CONFIRMATION_FAILED", detail)
        return False

    def pick_result(self, success: bool, detail: str = "", failure_code: int = 0) -> None:
        if self.state is not HarvestState.PICKING or self.current_target_id is None:
            raise ValueError("pick result has no active target")
        target_id = self.current_target_id
        if success:
            self.harvested.append(target_id)
            self.retry_target_id = None
            self.current_target_id = None
            self.state = HarvestState.SCANNING
            self.history.append(HarvestEvent(self.state, target_id, "HARVESTED", detail))
            if len(self.completed_ids) >= self.max_targets:
                self.finish("MAX_TARGETS_REACHED")
            return
        self.failures.append({"target_id": target_id, "failure_code": int(failure_code), "message": detail})
        self._attempt_failed("PICK_FAILED", detail)

    def begin_bounded_reobservation(
        self,
        outcome: str,
        detail: str = "",
        failure_code: int | None = None,
    ) -> bool:
        """Consume the one retry without losing an already observed target.

        Base-camera reacquisition can be temporarily blocked by the arm at an
        eye-in-hand observation pose.  When the orchestrator still owns a
        distinct, precomputed wrist view, keep the target identity and begin
        the second attempt directly.  This method never creates a third
        attempt; an exhausted target is skipped through the normal path.
        """

        if self.state not in {HarvestState.CONFIRMING, HarvestState.PICKING}:
            raise ValueError("bounded reobservation requires a confirmed observation")
        target_id = self.current_target_id
        if target_id is None:
            raise RuntimeError("bounded reobservation has no target")
        event_prefix = str(outcome).strip().upper()
        if not event_prefix:
            raise ValueError("bounded reobservation outcome must not be empty")
        if failure_code is not None:
            self.failures.append(
                {
                    "target_id": target_id,
                    "failure_code": int(failure_code),
                    "message": detail,
                }
            )
        if self.attempts[target_id] >= self.max_attempts_per_target:
            self._attempt_failed(event_prefix, detail)
            return False
        self.attempts[target_id] += 1
        self.retry_target_id = None
        self.state = HarvestState.OBSERVING
        self.history.append(
            HarvestEvent(
                self.state,
                target_id,
                f"{event_prefix}_REOBSERVATION",
                detail,
            )
        )
        return True

    def _attempt_failed(self, outcome: str, detail: str) -> None:
        target_id = self.current_target_id
        if target_id is None:
            raise RuntimeError("failed attempt has no target")
        if self.attempts[target_id] < self.max_attempts_per_target:
            self.retry_target_id = target_id
            recorded_outcome = f"{outcome}_RETRY"
        else:
            self.skipped[target_id] = detail or outcome
            self.retry_target_id = None
            recorded_outcome = f"{outcome}_SKIPPED"
        self.current_target_id = None
        self.state = HarvestState.SCANNING
        self.history.append(HarvestEvent(self.state, target_id, recorded_outcome, detail))
        if len(self.completed_ids) >= self.max_targets:
            self.finish("MAX_TARGETS_REACHED")

    def retry_unavailable(self, detail: str) -> int:
        """Skip a reserved retry that cannot be re-observed before timeout."""

        if self.state is not HarvestState.SCANNING or self.retry_target_id is None:
            raise ValueError("no reserved retry is waiting for a target")
        target_id = self.retry_target_id
        self.attempts[target_id] = self.max_attempts_per_target
        self.skipped[target_id] = detail or "retry target unavailable"
        self.retry_target_id = None
        self.history.append(
            HarvestEvent(
                self.state,
                target_id,
                "RETRY_UNAVAILABLE_SKIPPED",
                detail,
            )
        )
        if len(self.completed_ids) >= self.max_targets:
            self.finish("MAX_TARGETS_REACHED")
        return target_id

    def finish(self, detail: str = "") -> str:
        if self.state not in {HarvestState.SCANNING, HarvestState.DONE}:
            raise ValueError("sequence can finish only between target attempts")
        self.state = HarvestState.DONE
        if self.harvested and self.skipped:
            outcome = "PARTIAL_SUCCESS"
        elif self.harvested:
            outcome = "SUCCESS"
        elif self.skipped:
            outcome = "FAILED"
        else:
            outcome = "NO_PICK"
        self.history.append(HarvestEvent(self.state, None, outcome, detail))
        return outcome
