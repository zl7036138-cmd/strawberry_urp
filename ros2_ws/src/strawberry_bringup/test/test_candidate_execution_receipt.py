import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.candidate_execution_receipt import (  # noqa: E402
    qualify_adaptive_candidate_execution_receipt,
)


def event(event_type, time_ns, payload):
    return {
        "event_type": event_type,
        "event_time_monotonic_ns": time_ns,
        "payload": payload,
    }


class CandidateExecutionReceiptTests(unittest.TestCase):
    def _receipt(self):
        target_id = 7
        return {
            "action_result": {"success": True, "failure_code": 0},
            "motion_events": [
                event(
                    "ADAPTIVE_CANDIDATE_EXECUTION_STARTED",
                    10,
                    {
                        "target_id": target_id,
                        "live_payload_state": "EMPTY",
                        "controller_commands_before_authorization": 0,
                        "gripper_commands_before_authorization": 0,
                        "physical_attach_before_authorization": 0,
                    },
                ),
                event(
                    "CANDIDATE_QUALIFICATION_STARTED",
                    20,
                    {
                        "target_id": target_id,
                        "mode": "PRE_EXECUTION_QUALIFICATION",
                        "live_payload_state": "EMPTY",
                        "controller_commands_during_evaluation": 0,
                        "gripper_commands_during_evaluation": 0,
                        "physical_attach_during_evaluation": 0,
                    },
                ),
                event(
                    "CANDIDATE_EVALUATION_RESULT",
                    30,
                    {
                        "target_id": target_id,
                        "candidate_id": "G00",
                        "geometry_fingerprint": "fp0",
                        "result": "APPROACH_FAILED",
                        "feasible": False,
                    },
                ),
                event(
                    "CANDIDATE_EVALUATION_RESULT",
                    40,
                    {
                        "target_id": target_id,
                        "candidate_id": "G01",
                        "geometry_fingerprint": "fp1",
                        "result": "ESCAPE_FAILED",
                        "feasible": False,
                    },
                ),
                event(
                    "CANDIDATE_EVALUATION_RESULT",
                    50,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02",
                        "geometry_fingerprint": "fp2",
                        "result": "FEASIBLE",
                        "feasible": True,
                    },
                ),
                event(
                    "CANDIDATE_QUALIFICATION_RESULT",
                    60,
                    {
                        "target_id": target_id,
                        "mode": "PRE_EXECUTION_QUALIFICATION",
                        "status": "PLAN_ONLY_CERTIFIED",
                        "feasible": True,
                        "scene_isolated": True,
                        "selected_candidate_id": "G02",
                        "geometry_fingerprint": "fp2",
                        "certificate_fingerprint": "cert2",
                        "scene_signature_before": "scene2",
                        "execution_dispatched": False,
                    },
                ),
                event(
                    "TARGET_COLLISION_RESTORED",
                    70,
                    {"target_id": target_id, "restored": True},
                ),
                event(
                    "ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP",
                    80,
                    {
                        "target_id": target_id,
                        "target_collision_restored": True,
                        "execution_dispatched": False,
                    },
                ),
                event(
                    "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED",
                    90,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02",
                        "geometry_fingerprint": "fp2",
                        "certificate_fingerprint": "cert2",
                        "scene_signature": "scene2",
                        "execution_dispatched": True,
                    },
                ),
                event("COMMAND_PREPARED", 100, {"command_id": "arm-1"}),
                event(
                    "AUTHORIZED_CANDIDATE_EXECUTION_RESULT",
                    110,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02",
                        "geometry_fingerprint": "fp2",
                        "certificate_fingerprint": "cert2",
                        "success": True,
                    },
                ),
            ],
        }

    def test_valid_later_candidate_execution_passes(self):
        result = qualify_adaptive_candidate_execution_receipt(
            self._receipt(), target_id=7
        )
        self.assertTrue(result.passed)
        self.assertEqual(result.selected_candidate_id, "G02")
        self.assertEqual(result.certificate_fingerprint, "cert2")

    def test_command_before_dispatch_fails(self):
        receipt = self._receipt()
        receipt["motion_events"].insert(
            2, event("COMMAND_PREPARED", 25, {"command_id": "bad"})
        )
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn(
            "controller command occurred before candidate dispatch", result.errors
        )

    def test_dispatch_fingerprint_mismatch_fails(self):
        receipt = self._receipt()
        receipt["motion_events"][8]["payload"]["geometry_fingerprint"] = "other"
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("dispatch geometry_fingerprint differs from certificate", result.errors)

    def test_nominal_candidate_does_not_close_adaptive_evidence(self):
        receipt = self._receipt()
        qualification = receipt["motion_events"][5]["payload"]
        qualification["selected_candidate_id"] = "G00"
        qualification["geometry_fingerprint"] = "fp0"
        qualification["certificate_fingerprint"] = "cert0"
        for index in (8, 10):
            receipt["motion_events"][index]["payload"]["candidate_id"] = "G00"
            receipt["motion_events"][index]["payload"]["geometry_fingerprint"] = "fp0"
            receipt["motion_events"][index]["payload"]["certificate_fingerprint"] = "cert0"
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("adaptive execution did not replace nominal G00", result.errors)

    def test_action_failure_fails(self):
        receipt = self._receipt()
        receipt["action_result"]["success"] = False
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("action result was not successful", result.errors)


if __name__ == "__main__":
    unittest.main()
