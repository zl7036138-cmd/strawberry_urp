"""Read-only, whole-chain grasp feasibility contract (ADR 0086).

This module deliberately contains no ROS actions, controllers, gripper calls,
or payload-lifecycle transitions.  It coordinates a sequence of *hypothetical*
planning-scene states supplied by an adapter and produces an auditable
certificate for the nominal grasp already used by the executor.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Any, Protocol


class ChainFailureCode(str, Enum):
    """Stable rejection reasons for the pre-close authorization boundary."""

    FEASIBLE = "FEASIBLE"
    PREGRASP_FAILED = "PREGRASP_FAILED"
    APPROACH_FAILED = "APPROACH_FAILED"
    GRASP_STATE_INVALID = "GRASP_STATE_INVALID"
    ESCAPE_FAILED = "ESCAPE_FAILED"
    TRANSPORT_FAILED = "TRANSPORT_FAILED"
    BIN_APPROACH_FAILED = "BIN_APPROACH_FAILED"
    SCENE_INVALID = "SCENE_INVALID"
    TIME_BUDGET_EXCEEDED = "TIME_BUDGET_EXCEEDED"


@dataclass(frozen=True)
class StageAssessment:
    """Result of one hypothetical stage and its propagated terminal state."""

    feasible: bool
    collision: bool = False
    planning_time_sec: float = 0.0
    joint_travel_rad: float = 0.0
    terminal_state: Any = None
    detail: str = ""

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.planning_time_sec)
            or self.planning_time_sec < 0.0
            or not math.isfinite(self.joint_travel_rad)
            or self.joint_travel_rad < 0.0
        ):
            raise ValueError("stage metrics must be finite and non-negative")


@dataclass(frozen=True)
class WholeChainRequest:
    """One existing nominal grasp evaluated without moving the robot."""

    target_id: int
    target_pose: Any
    pregrasp_pose: Any
    grasp_pose: Any
    escape_pose: Any
    bin_pose: Any
    time_budget_sec: float = 8.0

    def __post_init__(self) -> None:
        if not isinstance(self.target_id, int) or isinstance(self.target_id, bool):
            raise ValueError("target_id must be an integer")
        if self.target_id <= 0:
            raise ValueError("target_id must be positive")
        if not math.isfinite(self.time_budget_sec) or self.time_budget_sec <= 0.0:
            raise ValueError("time budget must be finite and positive")


@dataclass(frozen=True)
class ChainEvaluation:
    """A feasibility certificate, never an executable trajectory."""

    code: ChainFailureCode
    planning_time_sec: float
    joint_travel_rad: float
    stages: tuple[str, ...] = ()
    detail: str = ""

    @property
    def feasible(self) -> bool:
        return self.code is ChainFailureCode.FEASIBLE


class WholeChainBackend(Protocol):
    """Adapter contract for an immutable temporary planning scene.

    Each successful method must return the terminal state that becomes the
    input of the next method.  ``virtual_attach`` returns a new state; it must
    not alter the live scene or the physical payload lifecycle.
    """

    def snapshot(self, request: WholeChainRequest) -> Any: ...

    def scene_is_valid(self, state: Any) -> bool: ...

    def pregrasp(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def approach(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def grasp_state(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def virtual_attach(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def escape(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def transport(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...

    def bin_approach(self, state: Any, request: WholeChainRequest) -> StageAssessment: ...


@dataclass
class WholeChainEvaluator:
    """Coordinate the authorization chain on an adapter-owned scene snapshot."""

    backend: WholeChainBackend
    clock: callable = field(default=time.perf_counter, repr=False)

    def evaluate(self, request: WholeChainRequest) -> ChainEvaluation:
        started = self.clock()
        planning_time = 0.0
        joint_travel = 0.0
        stages: list[str] = []

        def elapsed() -> float:
            value = float(self.clock() - started)
            return max(0.0, value)

        def result(code: ChainFailureCode, detail: str = "") -> ChainEvaluation:
            return ChainEvaluation(
                code,
                planning_time,
                joint_travel,
                tuple(stages),
                detail,
            )

        def over_budget() -> bool:
            return elapsed() > request.time_budget_sec

        try:
            state = self.backend.snapshot(request)
            if state is None or not self.backend.scene_is_valid(state):
                return result(ChainFailureCode.SCENE_INVALID, "scene snapshot is invalid")
            if over_budget():
                return result(ChainFailureCode.TIME_BUDGET_EXCEEDED)

            plan = (
                ("PREGRASP", ChainFailureCode.PREGRASP_FAILED, self.backend.pregrasp),
                ("APPROACH", ChainFailureCode.APPROACH_FAILED, self.backend.approach),
                ("GRASP_STATE", ChainFailureCode.GRASP_STATE_INVALID, self.backend.grasp_state),
                ("VIRTUAL_ATTACH", ChainFailureCode.SCENE_INVALID, self.backend.virtual_attach),
                ("ESCAPE", ChainFailureCode.ESCAPE_FAILED, self.backend.escape),
                ("TRANSPORT", ChainFailureCode.TRANSPORT_FAILED, self.backend.transport),
                ("BIN_APPROACH", ChainFailureCode.BIN_APPROACH_FAILED, self.backend.bin_approach),
            )
            for stage_name, failure, operation in plan:
                assessment = operation(state, request)
                planning_time += assessment.planning_time_sec
                joint_travel += assessment.joint_travel_rad
                stages.append(stage_name)
                if not assessment.feasible or assessment.terminal_state is None:
                    detail = assessment.detail or f"{stage_name.lower()} is infeasible"
                    return result(failure, detail)
                state = assessment.terminal_state
                if not self.backend.scene_is_valid(state):
                    return result(ChainFailureCode.SCENE_INVALID, f"{stage_name.lower()} invalidated scene")
                if over_budget():
                    return result(ChainFailureCode.TIME_BUDGET_EXCEEDED)
        except Exception as exc:
            # A planning-scene clone/IK failure must fail closed at the
            # authorization boundary; it must never fall through to closure.
            return result(ChainFailureCode.SCENE_INVALID, str(exc))
        return result(ChainFailureCode.FEASIBLE, "all nominal grasp stages are feasible")
