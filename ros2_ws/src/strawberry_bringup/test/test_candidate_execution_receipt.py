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
                    "ADAPTIVE_COLLISION_SCENE_LEASE_STARTED",
                    5,
                    {
                        "target_id": target_id,
                        "maximum_duration_sec": 600.0,
                        "obstacle_count": 4,
                        "scene_signature": "lease-scene",
                        "truth_source": False,
                    },
                ),
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
                event(
                    "ADAPTIVE_COLLISION_SCENE_LEASE_ENDED",
                    115,
                    {"target_id": target_id, "released": True},
                ),
            ],
        }

    def _receipt_with_centering(self):
        receipt = self._receipt()
        target_id = 7
        initial_result = next(
            row["payload"]
            for row in receipt["motion_events"]
            if row["event_type"] == "AUTHORIZED_CANDIDATE_EXECUTION_RESULT"
        )
        initial_result.update(
            {
                "success": False,
                "failure_code": "GRASP_FAILED",
                "payload_state": "EMPTY",
                "recovery_disposition": "AT_HOME",
                "contact_centering_source_candidate_id": "G02",
                "contact_centering_source_geometry_fingerprint": "fp2",
                "contact_centering_contact_class": "RIGHT_SINGLE_FRUIT",
                "contact_centering_local_y_offset_m": 0.004,
            }
        )
        receipt["motion_events"].extend(
            [
                event(
                    "CONTACT_CENTERING_REAUTHORIZATION_STARTED",
                    120,
                    {
                        "target_id": target_id,
                        "reauthorization_attempt": 1,
                        "maximum_reauthorizations": 1,
                        "source_candidate_id": "G02",
                        "source_geometry_fingerprint": "fp2",
                        "contact_class": "RIGHT_SINGLE_FRUIT",
                        "local_y_offset_m": 0.004,
                        "corrected_candidate_id": "G02-C01",
                        "corrected_geometry_fingerprint": "fp2c",
                        "corrected_candidate_ids": ["G02-C01"],
                        "corrected_geometry_fingerprints": ["fp2c"],
                        "correction_scale_factors": [1.0],
                        "live_payload_state": "EMPTY",
                        "recovery_disposition": "AT_HOME",
                    },
                ),
                event(
                    "CONTACT_CENTERING_QUALIFICATION_STARTED",
                    130,
                    {
                        "target_id": target_id,
                        "mode": "CONTACT_CENTERING_REAUTHORIZATION",
                        "live_payload_state": "EMPTY",
                        "required_first_candidate_id": "G02-C01",
                        "candidate_count": 1,
                        "candidate_ids": ["G02-C01"],
                        "candidate_geometry_fingerprints": ["fp2c"],
                        "controller_commands_during_evaluation": 0,
                        "gripper_commands_during_evaluation": 0,
                        "physical_attach_during_evaluation": 0,
                    },
                ),
                event(
                    "CONTACT_CENTERING_EVALUATION_RESULT",
                    140,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02-C01",
                        "geometry_fingerprint": "fp2c",
                        "result": "FEASIBLE",
                        "feasible": True,
                    },
                ),
                event(
                    "CONTACT_CENTERING_QUALIFICATION_RESULT",
                    150,
                    {
                        "target_id": target_id,
                        "mode": "CONTACT_CENTERING_REAUTHORIZATION",
                        "status": "PLAN_ONLY_CERTIFIED",
                        "feasible": True,
                        "scene_isolated": True,
                        "selected_candidate_id": "G02-C01",
                        "geometry_fingerprint": "fp2c",
                        "certificate_fingerprint": "cert2c",
                        "scene_signature_before": "scene2c",
                        "execution_dispatched": False,
                    },
                ),
                event(
                    "TARGET_COLLISION_RESTORED",
                    160,
                    {"target_id": target_id, "restored": True},
                ),
                event(
                    "CONTACT_CENTERING_REAUTHORIZATION_CLEANUP",
                    170,
                    {
                        "target_id": target_id,
                        "target_collision_restored": True,
                        "execution_dispatched": False,
                    },
                ),
                event(
                    "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED",
                    180,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02-C01",
                        "geometry_fingerprint": "fp2c",
                        "certificate_fingerprint": "cert2c",
                        "scene_signature": "scene2c",
                        "execution_dispatched": True,
                    },
                ),
                event("COMMAND_PREPARED", 190, {"command_id": "arm-2"}),
                event(
                    "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT",
                    200,
                    {
                        "target_id": target_id,
                        "candidate_id": "G02-C01",
                        "geometry_fingerprint": "fp2c",
                        "certificate_fingerprint": "cert2c",
                        "success": True,
                        "further_reauthorization_permitted": False,
                    },
                ),
            ]
        )
        lease_end = next(
            row
            for row in receipt["motion_events"]
            if row["event_type"] == "ADAPTIVE_COLLISION_SCENE_LEASE_ENDED"
        )
        lease_end["event_time_monotonic_ns"] = 210
        receipt["motion_events"].remove(lease_end)
        receipt["motion_events"].append(lease_end)
        return receipt

    def test_valid_later_candidate_execution_passes(self):
        result = qualify_adaptive_candidate_execution_receipt(
            self._receipt(), target_id=7
        )
        self.assertTrue(result.passed)
        self.assertEqual(result.selected_candidate_id, "G02")
        self.assertEqual(result.certificate_fingerprint, "cert2")

    def test_valid_single_contact_reauthorization_passes(self):
        result = qualify_adaptive_candidate_execution_receipt(
            self._receipt_with_centering(), target_id=7
        )
        self.assertTrue(result.passed, result.errors)
        self.assertEqual(result.selected_candidate_id, "G02-C01")
        self.assertEqual(result.certificate_fingerprint, "cert2c")

    def test_valid_partial_centering_search_passes(self):
        receipt = self._receipt_with_centering()
        events = receipt["motion_events"]
        start = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_REAUTHORIZATION_STARTED"
        )
        start["corrected_candidate_ids"] = ["G02-C01", "G02-C02"]
        start["corrected_geometry_fingerprints"] = ["fp2c", "fp2c2"]
        start["correction_scale_factors"] = [1.0, 0.75]
        qualification_start = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_QUALIFICATION_STARTED"
        )
        qualification_start["candidate_count"] = 2
        qualification_start["candidate_ids"] = ["G02-C01", "G02-C02"]
        qualification_start["candidate_geometry_fingerprints"] = [
            "fp2c",
            "fp2c2",
        ]
        first_evaluation_index = next(
            index
            for index, row in enumerate(events)
            if row["event_type"] == "CONTACT_CENTERING_EVALUATION_RESULT"
        )
        events[first_evaluation_index]["payload"].update(
            {"result": "TRANSPORT_FAILED", "feasible": False}
        )
        events.insert(
            first_evaluation_index + 1,
            event(
                "CONTACT_CENTERING_EVALUATION_RESULT",
                145,
                {
                    "target_id": 7,
                    "candidate_id": "G02-C02",
                    "geometry_fingerprint": "fp2c2",
                    "result": "FEASIBLE",
                    "feasible": True,
                },
            ),
        )
        qualification_result = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_QUALIFICATION_RESULT"
        )
        qualification_result["selected_candidate_id"] = "G02-C02"
        qualification_result["geometry_fingerprint"] = "fp2c2"
        qualification_result["certificate_fingerprint"] = "cert2c2"
        for event_type in (
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED",
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT",
        ):
            payload = next(
                row["payload"] for row in events if row["event_type"] == event_type
            )
            payload["candidate_id"] = "G02-C02"
            payload["geometry_fingerprint"] = "fp2c2"
            payload["certificate_fingerprint"] = "cert2c2"

        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)

        self.assertTrue(result.passed, result.errors)
        self.assertEqual(result.selected_candidate_id, "G02-C02")
        self.assertEqual(result.certificate_fingerprint, "cert2c2")

    def test_valid_next_untried_orientation_reauthorization_passes(self):
        receipt = self._receipt_with_centering()
        events = receipt["motion_events"]
        initial_qualification = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CANDIDATE_QUALIFICATION_STARTED"
        )
        initial_qualification["candidate_ids"] = [
            "G00", "G01", "G02", "G03", "G04"
        ]
        start = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_REAUTHORIZATION_STARTED"
        )
        start.update(
            {
                "reauthorization_strategy": "NEXT_UNTRIED_ORIENTATION",
                "corrected_candidate_id": "G03",
                "corrected_geometry_fingerprint": "fp3",
                "corrected_candidate_ids": ["G03", "G04"],
                "corrected_geometry_fingerprints": ["fp3", "fp4"],
                "correction_scale_factors": [],
                "escape_rejoins_source_candidate": False,
            }
        )
        qualification_start = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_QUALIFICATION_STARTED"
        )
        qualification_start.update(
            {
                "required_first_candidate_id": "G03",
                "candidate_count": 2,
                "candidate_ids": ["G03", "G04"],
                "candidate_geometry_fingerprints": ["fp3", "fp4"],
            }
        )
        first_evaluation = next(
            row
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_EVALUATION_RESULT"
        )
        first_evaluation["payload"].update(
            {
                "candidate_id": "G03",
                "geometry_fingerprint": "fp3",
                "result": "APPROACH_FAILED",
                "feasible": False,
            }
        )
        events.append(
            event(
                "CONTACT_CENTERING_EVALUATION_RESULT",
                145,
                {
                    "target_id": 7,
                    "candidate_id": "G04",
                    "geometry_fingerprint": "fp4",
                    "result": "FEASIBLE",
                    "feasible": True,
                },
            )
        )
        qualification_result = next(
            row["payload"]
            for row in events
            if row["event_type"] == "CONTACT_CENTERING_QUALIFICATION_RESULT"
        )
        qualification_result.update(
            {
                "selected_candidate_id": "G04",
                "geometry_fingerprint": "fp4",
                "certificate_fingerprint": "cert4",
            }
        )
        for event_type in (
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED",
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT",
        ):
            payload = next(
                row["payload"]
                for row in events
                if row["event_type"] == event_type
            )
            payload.update(
                {
                    "candidate_id": "G04",
                    "geometry_fingerprint": "fp4",
                    "certificate_fingerprint": "cert4",
                }
            )

        result = qualify_adaptive_candidate_execution_receipt(
            receipt, target_id=7
        )

        self.assertTrue(result.passed, result.errors)
        self.assertEqual(result.selected_candidate_id, "G04")
        self.assertEqual(result.certificate_fingerprint, "cert4")

    def test_centering_command_before_second_certificate_fails(self):
        receipt = self._receipt_with_centering()
        receipt["motion_events"].insert(
            -2,
            event("COMMAND_PREPARED", 145, {"command_id": "bad-centering"}),
        )
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn(
            "controller command occurred during centering qualification",
            result.errors,
        )

    def test_second_centering_attempt_fails_bounded_receipt(self):
        receipt = self._receipt_with_centering()
        receipt["motion_events"].append(
            event(
                "CONTACT_CENTERING_REAUTHORIZATION_STARTED",
                210,
                {
                    "target_id": 7,
                    "reauthorization_attempt": 2,
                },
            )
        )
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn(
            "expected exactly one CONTACT_CENTERING_REAUTHORIZATION_STARTED event for target",
            result.errors,
        )

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
        dispatch = next(
            row
            for row in receipt["motion_events"]
            if row["event_type"] == "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED"
        )
        dispatch["payload"]["geometry_fingerprint"] = "other"
        result = qualify_adaptive_candidate_execution_receipt(receipt, target_id=7)
        self.assertFalse(result.passed)
        self.assertIn("dispatch geometry_fingerprint differs from certificate", result.errors)

    def test_nominal_candidate_does_not_close_adaptive_evidence(self):
        receipt = self._receipt()
        qualification = next(
            row["payload"]
            for row in receipt["motion_events"]
            if row["event_type"] == "CANDIDATE_QUALIFICATION_RESULT"
        )
        qualification["selected_candidate_id"] = "G00"
        qualification["geometry_fingerprint"] = "fp0"
        qualification["certificate_fingerprint"] = "cert0"
        for row in receipt["motion_events"]:
            if row["event_type"] in {
                "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED",
                "AUTHORIZED_CANDIDATE_EXECUTION_RESULT",
            }:
                row["payload"]["candidate_id"] = "G00"
                row["payload"]["geometry_fingerprint"] = "fp0"
                row["payload"]["certificate_fingerprint"] = "cert0"
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
