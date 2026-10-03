import pathlib
import sys
import unittest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.whole_chain_qualification import qualify_whole_chain_receipt


def event(kind, time_ns, payload=None):
    return {"event_type": kind, "event_time_monotonic_ns": time_ns,
            "payload": payload or {}}


def payload(result):
    return {"result": result, "live_payload_state": "EMPTY", "scene_isolated": True,
            "controller_commands_during_evaluation": 0,
            "gripper_commands_during_evaluation": 0,
            "physical_attach_during_evaluation": 0}


class WholeChainQualificationTests(unittest.TestCase):
    def test_feasible_certificate_precedes_first_command(self):
        receipt = {"motion_events": [
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED")),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, payload("FEASIBLE")),
            event("COMMAND_PREPARED", 30),
        ]}
        result = qualify_whole_chain_receipt(receipt, expected_result="FEASIBLE", expect_motion=True)
        self.assertTrue(result.passed)
        self.assertEqual(result.first_command_after_evaluation_ns, 30)

    def test_transport_rejection_allows_no_command(self):
        receipt = {"motion_events": [
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED")),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, payload("TRANSPORT_FAILED")),
        ]}
        result = qualify_whole_chain_receipt(receipt, expected_result="TRANSPORT_FAILED", expect_motion=False)
        self.assertTrue(result.passed)

    def test_rejected_target_requires_later_explicit_collision_restore(self):
        restored = event(
            "TARGET_COLLISION_RESTORED", 30,
            {"target_id": 7, "restored": True},
        )
        receipt = {"motion_events": [
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED") | {"target_id": 7}),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, payload("TRANSPORT_FAILED") | {"target_id": 7}),
            restored,
        ]}
        result = qualify_whole_chain_receipt(
            receipt, expected_result="TRANSPORT_FAILED", expect_motion=False,
            target_id=7, require_target_collision_restore=True,
        )
        self.assertTrue(result.passed)

    def test_rejected_target_restore_must_follow_evaluation(self):
        receipt = {"motion_events": [
            event("TARGET_COLLISION_RESTORED", 5, {"target_id": 7, "restored": True}),
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED") | {"target_id": 7}),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, payload("TRANSPORT_FAILED") | {"target_id": 7}),
        ]}
        result = qualify_whole_chain_receipt(
            receipt, expected_result="TRANSPORT_FAILED", expect_motion=False,
            target_id=7, require_target_collision_restore=True,
        )
        self.assertFalse(result.passed)
        self.assertIn("target collision was not restored after rejection", result.errors)

    def test_target_id_selects_one_attempt_from_a_continuous_harvest(self):
        first = payload("PREGRASP_FAILED") | {"target_id": 4}
        second = payload("FEASIBLE") | {"target_id": 5}
        receipt = {"motion_events": [
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED") | {"target_id": 4}),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, first),
            event("COMMAND_PREPARED", 25),  # independent recovery after target 4
            event("WHOLE_CHAIN_EVALUATION_STARTED", 30, payload("STARTED") | {"target_id": 5}),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 40, second),
            event("COMMAND_PREPARED", 50),
        ]}
        result = qualify_whole_chain_receipt(
            receipt, expected_result="FEASIBLE", expect_motion=True, target_id=5
        )
        self.assertTrue(result.passed)
        self.assertEqual(result.first_command_after_evaluation_ns, 50)

    def test_command_during_evaluation_fails(self):
        receipt = {"motion_events": [
            event("WHOLE_CHAIN_EVALUATION_STARTED", 10, payload("STARTED")),
            event("COMMAND_PREPARED", 15),
            event("WHOLE_CHAIN_EVALUATION_RESULT", 20, payload("FEASIBLE")),
        ]}
        result = qualify_whole_chain_receipt(receipt, expected_result="FEASIBLE", expect_motion=True)
        self.assertFalse(result.passed)
        self.assertIn("controller/gripper command occurred during evaluation", result.errors)


if __name__ == "__main__":
    unittest.main()
