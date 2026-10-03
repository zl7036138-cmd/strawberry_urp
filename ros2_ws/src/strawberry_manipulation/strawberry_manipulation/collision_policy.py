"""Phase-aware collision semantics with fail-closed defaults.

MoveIt continues to enforce every collision object as hard today.  This module
separates that conservative implementation from the semantic policy needed for
future plant-leaf and stem models: no caller can silently turn a hard collision
into a permitted contact by changing a route stage name.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class CollisionSemantic(str, Enum):
    HARD = "HARD"
    SOFT = "SOFT"
    TASK = "TASK"


class CollisionPhase(str, Enum):
    TRANSIT = "TRANSIT"
    GRASP_CONTACT = "GRASP_CONTACT"
    ESCAPE = "ESCAPE"
    BIN_ENTRY = "BIN_ENTRY"
    RETURN_ROUTE = "RETURN_ROUTE"
    HOME = "HOME"


@dataclass(frozen=True)
class CollisionRule:
    """One object class and its explicitly authorized physical contacts."""

    semantic: CollisionSemantic
    permitted_phases: frozenset[CollisionPhase] = frozenset()

    def permits(self, phase: CollisionPhase) -> bool:
        return phase in self.permitted_phases


# Hard geometry is never bypassed: table, planter, bin walls, field ridges,
# robot self-collision and non-selected fruit retain the existing MoveIt rules.
HARD_COLLISION_RULE = CollisionRule(CollisionSemantic.HARD)

# The current scene has no independently validated leaf/stem collision proxy.
# Keep future soft objects hard in practice until a bounded contact model and
# telemetry prove otherwise; this avoids using the semantic category as an
# implicit license to pass through vegetation.
SOFT_COLLISION_RULE = CollisionRule(CollisionSemantic.SOFT)

# The selected fruit is the sole task contact exception, and only the final
# grasp descent may open its collision object. It is restored as carried-body
# geometry immediately after attachment.
SELECTED_FRUIT_TASK_RULE = CollisionRule(
    CollisionSemantic.TASK,
    frozenset({CollisionPhase.GRASP_CONTACT}),
)


def selected_fruit_contact_is_authorized(phase: CollisionPhase) -> bool:
    """Return true only for the audited selected-fruit grasp contact phase."""

    return SELECTED_FRUIT_TASK_RULE.permits(phase)
