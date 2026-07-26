import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.core import (  # noqa: E402
    Candidate,
    Event,
    TargetSource,
    State,
    TrialStateMachine,
    parse_oracle_catalog,
    select_oracle_pose_index,
    select_target,
    validate_target_routing,
)


class TrialStateMachineTests(unittest.TestCase):
    def test_happy_path(self):
        machine = TrialStateMachine()
        for event in (
            Event.START,
            Event.READY,
            Event.FRAME,
            Event.DETECTIONS,
            Event.TARGET,
            Event.TARGET,
            Event.PLANNED,
            Event.APPROACHED,
            Event.GRASPED,
            Event.RETREATED,
            Event.PLACED,
            Event.VERIFIED,
        ):
            machine.apply(event)
        self.assertEqual(machine.state, State.DONE)

    def test_no_pick_is_safe_terminal(self):
        machine = TrialStateMachine()
        for event in (Event.START, Event.READY, Event.NO_PICK):
            machine.apply(event)
        self.assertEqual(machine.state, State.DONE)

    def test_sensor_retries_are_bounded(self):
        machine = TrialStateMachine(sensor_retry_limit=3)
        machine.apply(Event.START)
        machine.apply(Event.READY)
        for _ in range(4):
            machine.apply(Event.RETRY_SENSOR)
        self.assertEqual(machine.state, State.FAILED)
        self.assertEqual(machine.sensor_retries, 4)

    def test_invalid_transition_is_rejected(self):
        machine = TrialStateMachine()
        with self.assertRaises(ValueError):
            machine.apply(Event.GRASPED)

    def test_target_selection_contract(self):
        candidates = [
            Candidate(1, 2, 0.99, 0.2, True, True),
            Candidate(2, 1, 0.80, 0.3, True, True),
            Candidate(3, 1, 0.90, 0.5, True, True),
            Candidate(4, 1, 0.95, 0.1, False, True),
        ]
        selected = select_target(candidates)
        self.assertIsNotNone(selected)
        self.assertEqual(selected.target_id, 3)

    def test_target_selection_tie_breaks_by_distance(self):
        candidates = [
            Candidate(9, 1, 0.9, 0.5, True, True),
            Candidate(8, 1, 0.9, 0.3, True, True),
        ]
        self.assertEqual(select_target(candidates).target_id, 8)


class DualPathContractTests(unittest.TestCase):
    CATALOG = """{
      "schema_version": 1,
      "frame_id": "panda_link0",
      "pose_array_order": [1, 2, 3],
      "fruits": [
        {"target_id": 1, "maturity": "RIPE"},
        {"target_id": 2, "maturity": "UNRIPE"},
        {"target_id": 3, "maturity": "RIPE"}
      ]
    }"""

    def test_control_and_shadow_topics_are_separate(self):
        routing = validate_target_routing(
            source="oracle",
            control_topic="/strawberry/oracle/target_pose",
            shadow_enabled=True,
            shadow_topic="/strawberry/shadow/target_pose",
        )
        self.assertEqual(routing.source, TargetSource.ORACLE)

    def test_same_control_and_shadow_topic_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "must differ"):
            validate_target_routing(
                source="oracle",
                control_topic="/same",
                shadow_enabled=True,
                shadow_topic="/same",
            )

    def test_unknown_source_and_relative_topics_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "target_source"):
            validate_target_routing(
                source="truthish",
                control_topic="/control",
                shadow_enabled=False,
                shadow_topic="/shadow",
            )
        with self.assertRaisesRegex(ValueError, "absolute"):
            validate_target_routing(
                source="oracle",
                control_topic="control",
                shadow_enabled=False,
                shadow_topic="/shadow",
            )

    def test_oracle_provider_selects_only_ripe_targets(self):
        catalog = parse_oracle_catalog(self.CATALOG)
        self.assertEqual(select_oracle_pose_index(catalog, 3, 1), 0)
        self.assertEqual(select_oracle_pose_index(catalog, 3, 2), None)
        self.assertEqual(select_oracle_pose_index(catalog, 3, 0), 0)

    def test_oracle_catalog_identity_and_pose_count_fail_closed(self):
        catalog = parse_oracle_catalog(self.CATALOG)
        with self.assertRaisesRegex(ValueError, "pose count"):
            select_oracle_pose_index(catalog, 2, 1)
        with self.assertRaisesRegex(ValueError, "absent"):
            select_oracle_pose_index(catalog, 3, 99)


if __name__ == "__main__":
    unittest.main()
