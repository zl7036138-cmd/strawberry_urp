"""Small deterministic 3-D tracker shared by base and wrist observations."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Sequence


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _position(value: Sequence[float]) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError("position must contain x, y, z")
    return tuple(_finite(component, "position component") for component in value)


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


@dataclass(frozen=True)
class LocalizedObservation:
    source_detection_id: int
    maturity: int
    position: tuple[float, float, float]
    confidence: float
    sigma_m: float

    def __post_init__(self) -> None:
        if self.source_detection_id <= 0:
            raise ValueError("source_detection_id must be positive")
        if self.maturity not in {0, 1, 2}:
            raise ValueError("maturity must be UNKNOWN, RIPE, or UNRIPE")
        object.__setattr__(self, "position", _position(self.position))
        confidence = _finite(self.confidence, "confidence")
        sigma = _finite(self.sigma_m, "sigma_m")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if sigma <= 0.0:
            raise ValueError("sigma_m must be positive")


@dataclass(frozen=True)
class TrackedObservation:
    track_id: int
    source_detection_id: int
    maturity: int
    position: tuple[float, float, float]
    confidence: float
    sigma_m: float
    observation_count: int
    first_seen_sec: float
    last_seen_sec: float

    @property
    def stable(self) -> bool:
        return self.observation_count > 0


@dataclass
class _Track:
    track_id: int
    source_detection_id: int
    position: tuple[float, float, float]
    confidence: float
    sigma_m: float
    observation_count: int
    first_seen_sec: float
    last_seen_sec: float
    maturity_votes: dict[int, int] = field(default_factory=dict)
    latest_maturity: int = 0

    def maturity(self) -> int:
        maximum = max(self.maturity_votes.values())
        winners = [key for key, value in self.maturity_votes.items() if value == maximum]
        return self.latest_maturity if self.latest_maturity in winners else min(winners)


class MultiTargetTracker:
    """Nearest-neighbour tracker with one-to-one assignment and bounded age."""

    def __init__(
        self,
        *,
        association_distance_m: float = 0.06,
        completed_suppression_distance_m: float = 0.04,
        max_track_age_sec: float = 0.75,
        minimum_observations: int = 3,
        smoothing_alpha: float = 0.35,
    ) -> None:
        self.association_distance_m = _finite(association_distance_m, "association_distance_m")
        self.completed_suppression_distance_m = _finite(
            completed_suppression_distance_m,
            "completed_suppression_distance_m",
        )
        self.max_track_age_sec = _finite(max_track_age_sec, "max_track_age_sec")
        self.smoothing_alpha = _finite(smoothing_alpha, "smoothing_alpha")
        if self.association_distance_m <= 0.0 or self.max_track_age_sec <= 0.0:
            raise ValueError("tracker distance and age bounds must be positive")
        if not 0.0 < self.completed_suppression_distance_m < self.association_distance_m:
            raise ValueError(
                "completed suppression distance must be positive and below association distance"
            )
        if isinstance(minimum_observations, bool) or minimum_observations <= 0:
            raise ValueError("minimum_observations must be positive")
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        self.minimum_observations = int(minimum_observations)
        self._tracks: dict[int, _Track] = {}
        self._next_track_id = 1
        self._harvested: set[int] = set()
        self.identity_switches = 0
        self.association_count = 0

    def _prune(self, stamp_sec: float) -> None:
        # Fruit are stationary until attachment.  Keep dormant tracks as a
        # bounded-scene identity archive, but suppress them from snapshots
        # below once they are stale.  This lets a fruit regain its original ID
        # after a wrist-motion / GPU-processing gap and lets a late completion
        # event still create a geometric tombstone.
        _finite(stamp_sec, "stamp_sec")

    def update(
        self,
        observations: Iterable[LocalizedObservation],
        *,
        stamp_sec: float,
    ) -> tuple[TrackedObservation, ...]:
        stamp = _finite(stamp_sec, "stamp_sec")
        if stamp < 0.0:
            raise ValueError("stamp_sec must be non-negative")
        incoming = tuple(observations)
        self._prune(stamp)

        # Sort every admissible pair by distance, then IDs. Greedy selection is
        # deterministic and one-to-one; fruit are physically separated by more
        # than the association gate in generated scenes.
        pairs: list[tuple[float, int, int]] = []
        for observation_index, observation in enumerate(incoming):
            for track_id, track in self._tracks.items():
                # Completed tracks remain geometric tombstones.  Continue to
                # associate observations to them, but suppress them from every
                # snapshot below.  Otherwise an unpicked-but-skipped fruit is
                # immediately reborn with a new ID and defeats the one-retry
                # batch policy.
                distance = _distance(observation.position, track.position)
                distance_limit = (
                    self.completed_suppression_distance_m
                    if track_id in self._harvested
                    else self.association_distance_m
                )
                if distance <= distance_limit:
                    pairs.append((distance, track_id, observation_index))
        pairs.sort(key=lambda row: (row[0], row[1], incoming[row[2]].source_detection_id))
        assigned_tracks: set[int] = set()
        assigned_observations: set[int] = set()
        assignments: list[tuple[int, int]] = []
        for _distance_m, track_id, observation_index in pairs:
            if track_id in assigned_tracks or observation_index in assigned_observations:
                continue
            assigned_tracks.add(track_id)
            assigned_observations.add(observation_index)
            assignments.append((track_id, observation_index))

        for track_id, observation_index in assignments:
            observation = incoming[observation_index]
            track = self._tracks[track_id]
            alpha = self.smoothing_alpha
            track.position = tuple(
                alpha * observed + (1.0 - alpha) * previous
                for observed, previous in zip(observation.position, track.position)
            )
            track.confidence = alpha * observation.confidence + (1.0 - alpha) * track.confidence
            track.sigma_m = max(1e-6, alpha * observation.sigma_m + (1.0 - alpha) * track.sigma_m)
            track.source_detection_id = observation.source_detection_id
            track.observation_count += 1
            track.last_seen_sec = stamp
            track.latest_maturity = observation.maturity
            track.maturity_votes[observation.maturity] = track.maturity_votes.get(observation.maturity, 0) + 1
            self.association_count += 1

        for observation_index, observation in enumerate(incoming):
            if observation_index in assigned_observations:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = _Track(
                track_id=track_id,
                source_detection_id=observation.source_detection_id,
                position=observation.position,
                confidence=observation.confidence,
                sigma_m=observation.sigma_m,
                observation_count=1,
                first_seen_sec=stamp,
                last_seen_sec=stamp,
                maturity_votes={observation.maturity: 1},
                latest_maturity=observation.maturity,
            )

        return self.snapshot(stamp_sec=stamp)

    def snapshot(
        self,
        *,
        stamp_sec: float,
        stable_only: bool = False,
    ) -> tuple[TrackedObservation, ...]:
        stamp = _finite(stamp_sec, "stamp_sec")
        self._prune(stamp)
        result = []
        for track in self._tracks.values():
            if track.track_id in self._harvested:
                continue
            if stamp - track.last_seen_sec > self.max_track_age_sec:
                continue
            if stable_only and track.observation_count < self.minimum_observations:
                continue
            result.append(
                TrackedObservation(
                    track_id=track.track_id,
                    source_detection_id=track.source_detection_id,
                    maturity=track.maturity(),
                    position=track.position,
                    confidence=track.confidence,
                    sigma_m=track.sigma_m,
                    observation_count=track.observation_count,
                    first_seen_sec=track.first_seen_sec,
                    last_seen_sec=track.last_seen_sec,
                )
            )
        return tuple(sorted(result, key=lambda item: item.track_id))

    def mark_harvested(self, track_id: int) -> None:
        if track_id not in self._tracks:
            raise KeyError(track_id)
        self._harvested.add(track_id)

    def reset(self) -> None:
        """Forget camera-local tracks when the bounded attention target changes."""

        self._tracks.clear()
        self._next_track_id = 1
        self._harvested.clear()
        self.identity_switches = 0
        self.association_count = 0

    @property
    def identity_switch_rate(self) -> float:
        return self.identity_switches / self.association_count if self.association_count else 0.0
