"""Bounded first-feasible grasp search for ADR 0087-B.

The search has no ROS, MoveIt, scoring, or execution dependency.  It records
every attempted candidate and stops at the first ADR 0086 certificate that is
feasible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Callable, Protocol

from .core import Pose
from .grasp_candidates import GraspCandidate
from .whole_chain import ChainEvaluation, ChainFailureCode, WholeChainEvaluator, WholeChainRequest


class CandidateSearchStatus(str, Enum):
    FOUND = "CANDIDATE_FOUND"
    NO_FEASIBLE_CANDIDATE = "NO_FEASIBLE_CANDIDATE"
    SEARCH_TIME_BUDGET_EXCEEDED = "SEARCH_TIME_BUDGET_EXCEEDED"
    CANDIDATE_SET_INVALID = "CANDIDATE_SET_INVALID"


@dataclass(frozen=True)
class CandidateEvaluation:
    """One immutable trace row retained even when a later candidate wins."""

    candidate_id: str
    geometry_fingerprint: str
    result: ChainEvaluation
    allocated_budget_sec: float


@dataclass(frozen=True)
class CandidateSearchResult:
    status: CandidateSearchStatus
    selected_candidate: GraspCandidate | None
    evaluations: tuple[CandidateEvaluation, ...]
    elapsed_sec: float
    detail: str = ""

    def __post_init__(self) -> None:
        if self.status is CandidateSearchStatus.FOUND:
            if self.selected_candidate is None:
                raise ValueError("found result requires a selected candidate")
        elif self.selected_candidate is not None:
            raise ValueError("only found result may select a candidate")


class CandidateEvaluator(Protocol):
    def evaluate(self, candidate: GraspCandidate, time_budget_sec: float) -> ChainEvaluation: ...


@dataclass(frozen=True)
class WholeChainCandidateEvaluator:
    """Pure adapter binding one target/bin context to ADR 0086 evaluation."""

    whole_chain_evaluator: WholeChainEvaluator
    target_id: int
    target_pose: Pose
    bin_pose: Pose

    def evaluate(self, candidate: GraspCandidate, time_budget_sec: float) -> ChainEvaluation:
        return self.whole_chain_evaluator.evaluate(
            WholeChainRequest(
                target_id=self.target_id,
                target_pose=self.target_pose,
                pregrasp_pose=candidate.pregrasp_pose,
                grasp_pose=candidate.grasp_pose,
                escape_pose=candidate.escape_pose,
                bin_pose=self.bin_pose,
                time_budget_sec=time_budget_sec,
            )
        )


@dataclass
class FirstFeasibleCandidateSearch:
    """Run candidates in frozen order subject to per- and total-budget caps."""

    evaluator: CandidateEvaluator
    per_candidate_budget_sec: float = 2.0
    total_budget_sec: float = 12.0
    clock: Callable[[], float] = field(default=time.perf_counter, repr=False)

    def __post_init__(self) -> None:
        for value, label in (
            (self.per_candidate_budget_sec, "per-candidate budget"),
            (self.total_budget_sec, "total budget"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be finite and positive")

    @staticmethod
    def _validate(candidates: tuple[GraspCandidate, ...]) -> str | None:
        if not candidates:
            return "candidate set is empty"
        ids = tuple(candidate.candidate_id for candidate in candidates)
        fingerprints = tuple(candidate.geometry_fingerprint for candidate in candidates)
        if len(ids) != len(set(ids)):
            return "candidate IDs must be unique"
        if len(fingerprints) != len(set(fingerprints)):
            return "candidate geometry fingerprints must be unique"
        if candidates[0].candidate_id != "G00":
            return "candidate set must begin with nominal G00"
        return None

    def search(self, candidates: tuple[GraspCandidate, ...]) -> CandidateSearchResult:
        invalid = self._validate(candidates)
        if invalid is not None:
            return CandidateSearchResult(
                CandidateSearchStatus.CANDIDATE_SET_INVALID, None, (), 0.0, invalid
            )
        started = float(self.clock())
        evaluations: list[CandidateEvaluation] = []

        def elapsed() -> float:
            return max(0.0, float(self.clock()) - started)

        for candidate in candidates:
            remaining = self.total_budget_sec - elapsed()
            if remaining <= 0.0:
                return CandidateSearchResult(
                    CandidateSearchStatus.SEARCH_TIME_BUDGET_EXCEEDED,
                    None,
                    tuple(evaluations),
                    elapsed(),
                    "candidate search total budget exhausted",
                )
            allocated = min(self.per_candidate_budget_sec, remaining)
            result = self.evaluator.evaluate(candidate, allocated)
            evaluations.append(
                CandidateEvaluation(
                    candidate.candidate_id,
                    candidate.geometry_fingerprint,
                    result,
                    allocated,
                )
            )
            # A misbehaving adapter cannot turn an over-budget search into a
            # valid certificate by returning FEASIBLE after the group deadline.
            if elapsed() > self.total_budget_sec:
                return CandidateSearchResult(
                    CandidateSearchStatus.SEARCH_TIME_BUDGET_EXCEEDED,
                    None,
                    tuple(evaluations),
                    elapsed(),
                    "candidate search total budget exhausted during evaluation",
                )
            if result.feasible:
                return CandidateSearchResult(
                    CandidateSearchStatus.FOUND,
                    candidate,
                    tuple(evaluations),
                    elapsed(),
                    "first feasible candidate selected",
                )
        return CandidateSearchResult(
            CandidateSearchStatus.NO_FEASIBLE_CANDIDATE,
            None,
            tuple(evaluations),
            elapsed(),
            "all bounded candidates were rejected",
        )
