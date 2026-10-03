import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.candidate_qualification_receipt import (  # noqa: E402
    qualify_candidate_plan_only_receipt,
)


def event(kind, time_ns, payload=None):
    return {
        "event_type": kind,
        "event_time_monotonic_ns": time_ns,
        "payload": payload or {},
    }


def start_payload():
    return {
        "target_id": 7,
        "mode": "PLAN_ONLY",
        "live_payload_state": "EMPTY",
        "candidate_ids": ["G00", "G01", "G02"],
        "candidate_geometry_fingerprints": ["fp0", "fp1", "fp2"],
        "controller_commands_during_evaluation": 0,
        "gripper_commands_during_evaluation": 0,
        "physical_attach_during_evaluation": 0,
    }


def evaluation(candidate_id, fingerprint, result, feasible):
    return {
        "target_id": 7,
        "candidate_id": candidate_id,
        "geometry_fingerprint": fingerprint,
        "result": result,
        "feasible": feasible,
    }


def result_payload():
    return {
        "target_id": 7,
        "status": "PLAN_ONLY_CERTIFIED",
        "feasible": True,
        "mode": "PLAN_ONLY",
        "live_payload_state": "EMPTY",
        "scene_isolated": True,
        "selected_candidate_id": "G02",
        "geometry_fingerprint": "fp2",
        "certificate_fingerprint": "cert2",
        "execution_dispatched": False,
        "controller_commands_during_evaluation": 0,
        "gripper_commands_during_evaluation": 0,
        "physical_attach_during_evaluation": 0,
    }


class CandidateQualificationReceiptTests(unittest.TestCase):
    def _receipt(self):
        return {"motion_events": [
            event("CANDIDATE_QUALIFICATION_STARTED", 10, start_payload()),
            event(
                "CANDIDATE_EVALUATION_RESULT", 20,
                evaluation("G00", "fp0", "ESCAPE_FAILED", False),
            ),
            event(
                "CANDIDATE_EVALUATION_RESULT", 30,
                evaluation("G01", "fp1", "TRANSPORT_FAILED", False),
            ),
            event(
                "CANDIDATE_EVALUATION_RESULT", 40,
                evaluation("G02", "fp2", "FEASIBLE", True),
            ),
            event("CANDIDATE_QUALIFICATION_RESULT", 50, result_payload()),
            event("TARGET_COLLISION_RESTORED", 60, {"target_id": 7, "restored": True}),
            event(
                "CANDIDATE_QUALIFICATION_CLEANUP", 70,
                {
                    "target_id": 7,
                    "mode": "PLAN_ONLY",
                    "target_collision_restored": True,
                    "execution_dispatched": False,
                },
            ),
        ]}

    def test_valid_first_feasible_plan_only_receipt_passes(self):
        result = qualify_candidate_plan_only_receipt(self._receipt(), target_id=7)
        self.assertTrue(result.passed)
        self.assertEqual(result.status, "PLAN_ONLY_CERTIFIED")
        self.assertEqual(result.selected_candidate_id, "G02")

    def test_motion_during_plan_only_interval_fails(self):
        receipt = self._receipt()
        receipt["motion_events"].insert(3, event("COMMAND_EXECUTED", 35))
        result = qualify_candidate_plan_only_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn(
            "plan-only qualification emitted motion/gripper/attach evidence",
            result.errors,
        )

    def test_selected_candidate_must_be_first_feasible(self):
        receipt = self._receipt()
        receipt["motion_events"][4]["payload"]["selected_candidate_id"] = "G01"
        result = qualify_candidate_plan_only_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("selected candidate differs from final feasible row", result.errors)

    def test_collision_restore_must_precede_cleanup(self):
        receipt = self._receipt()
        receipt["motion_events"].pop(5)
        result = qualify_candidate_plan_only_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("target collision was not restored after qualification", result.errors)

    def test_collision_restore_requires_explicit_success_boolean(self):
        receipt = self._receipt()
        receipt["motion_events"][5]["payload"].pop("restored")
        result = qualify_candidate_plan_only_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("target collision was not restored after qualification", result.errors)


if __name__ == "__main__":
    unittest.main()
