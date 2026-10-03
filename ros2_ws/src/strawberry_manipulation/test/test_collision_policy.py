import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.collision_policy import (  # noqa: E402
    CollisionPhase,
    HARD_COLLISION_RULE,
    SELECTED_FRUIT_TASK_RULE,
    SOFT_COLLISION_RULE,
    selected_fruit_contact_is_authorized,
)
from strawberry_manipulation.scene_geometry import (  # noqa: E402
    CollisionObjectSpec,
    BoxPrimitive,
    STATIC_COLLISION_OBJECTS,
)


class CollisionPolicyTests(unittest.TestCase):
    def test_hard_and_unvalidated_soft_contacts_are_never_authorized(self):
        for phase in CollisionPhase:
            self.assertFalse(HARD_COLLISION_RULE.permits(phase))
            self.assertFalse(SOFT_COLLISION_RULE.permits(phase))

    def test_selected_fruit_task_contact_is_limited_to_final_grasp_phase(self):
        self.assertTrue(selected_fruit_contact_is_authorized(CollisionPhase.GRASP_CONTACT))
        self.assertTrue(SELECTED_FRUIT_TASK_RULE.permits(CollisionPhase.GRASP_CONTACT))
        for phase in (
            CollisionPhase.TRANSIT,
            CollisionPhase.ESCAPE,
            CollisionPhase.BIN_ENTRY,
            CollisionPhase.RETURN_ROUTE,
            CollisionPhase.HOME,
        ):
            self.assertFalse(selected_fruit_contact_is_authorized(phase))

    def test_static_geometry_defaults_to_hard_collision_rule(self):
        for specification in STATIC_COLLISION_OBJECTS:
            self.assertEqual(specification.collision_rule, HARD_COLLISION_RULE)
        specification = CollisionObjectSpec(
            "test_hard_box",
            (BoxPrimitive((0.0, 0.0, 0.0), (0.1, 0.1, 0.1)),),
        )
        self.assertEqual(specification.collision_rule, HARD_COLLISION_RULE)


if __name__ == "__main__":
    unittest.main()
