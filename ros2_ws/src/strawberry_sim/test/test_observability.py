import unittest

from strawberry_sim.observability import (
    aggregate_observability_receipts,
    score_observability,
)


class ObservabilityScoringTests(unittest.TestCase):
    def test_distinguishes_view_detection_classification_and_localization(self):
        truth = [
            {
                "target_id": 1,
                "maturity": 1,
                "projected_center_in_image": True,
                "depth_visible": True,
                "containing_detection_indices": [0],
            },
            {
                "target_id": 2,
                "maturity": 1,
                "projected_center_in_image": True,
                "depth_visible": False,
                "containing_detection_indices": [],
            },
            {
                "target_id": 3,
                "maturity": 0,
                "projected_center_in_image": False,
                "depth_visible": False,
                "containing_detection_indices": [],
            },
        ]
        detections = [
            {
                "maturity": 1,
                "localization_status": "ACCEPTED",
                "nearest_truth_target_id": 1,
                "nearest_truth_error_m": 0.009,
            }
        ]
        scored, metrics = score_observability(truth, detections)
        self.assertEqual(
            [row["observability_status"] for row in scored],
            ["LOCALIZED", "DEPTH_OCCLUDED", "OUT_OF_IMAGE"],
        )
        self.assertEqual(metrics["localized_ripe_truth_count"], 1)
        self.assertEqual(metrics["ripe_localization_recall"], 0.5)
        self.assertEqual(metrics["visible_ripe_localization_recall"], 1.0)
        self.assertEqual(metrics["ripe_prediction_precision"], 1.0)

    def test_wrong_maturity_and_large_error_fail_closed(self):
        truth = [
            {
                "target_id": 4,
                "maturity": 1,
                "projected_center_in_image": True,
                "depth_visible": True,
                "containing_detection_indices": [0, 1],
            }
        ]
        detections = [
            {"maturity": 0, "localization_status": "REJECTED"},
            {
                "maturity": 1,
                "localization_status": "ACCEPTED",
                "nearest_truth_target_id": 4,
                "nearest_truth_error_m": 0.031,
            },
        ]
        scored, metrics = score_observability(truth, detections)
        self.assertEqual(scored[0]["observability_status"], "NO_ACCEPTED_LOCALIZATION")
        self.assertEqual(metrics["ripe_localization_recall"], 0.0)
        self.assertEqual(metrics["ripe_prediction_precision"], 0.0)

    def test_invalid_detection_reference_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "invalid detection index"):
            score_observability(
                [
                    {
                        "target_id": 1,
                        "maturity": 1,
                        "projected_center_in_image": True,
                        "depth_visible": True,
                        "containing_detection_indices": [2],
                    }
                ],
                [],
            )

    def test_aggregate_recomputes_scene_metrics(self):
        def receipt(target_id, error):
            return {
                "schema_version": 3,
                "kind": "generalized_rgbd_frame_diagnostic",
                "runtime_truth_use": False,
                "commands_published": 0,
                "truth_projection_scoring": [
                    {
                        "target_id": target_id,
                        "maturity": 1,
                        "projected_center_in_image": True,
                        "containing_detection_indices": [0],
                    }
                ],
                "detections": [
                    {
                        "maturity": 1,
                        "localization_status": "ACCEPTED",
                        "nearest_truth_target_id": target_id,
                        "nearest_truth_error_m": error,
                    }
                ],
            }

        result = aggregate_observability_receipts(
            [receipt(1, 0.01), receipt(2, 0.04)]
        )
        self.assertEqual(result["scene_count"], 2)
        self.assertEqual(result["localized_ripe_truth_count"], 1)
        self.assertEqual(result["ripe_localization_recall"], 0.5)
        self.assertEqual(result["visible_ripe_localization_recall"], 0.5)
        self.assertEqual(result["all_ripe_localized_scene_count"], 1)
        self.assertEqual(
            result["observability_status_counts"],
            {"LOCALIZED": 1, "NO_ACCEPTED_LOCALIZATION": 1},
        )

    def test_aggregate_rejects_runtime_truth_or_motion_receipt(self):
        with self.assertRaisesRegex(ValueError, "contract"):
            aggregate_observability_receipts(
                [
                    {
                        "schema_version": 3,
                        "kind": "generalized_rgbd_frame_diagnostic",
                        "runtime_truth_use": True,
                        "commands_published": 0,
                    }
                ]
            )

    def test_scene_without_visible_ripe_is_not_counted_as_complete(self):
        result = aggregate_observability_receipts(
            [
                {
                    "schema_version": 3,
                    "kind": "generalized_rgbd_frame_diagnostic",
                    "runtime_truth_use": False,
                    "commands_published": 0,
                    "truth_projection_scoring": [
                        {
                            "target_id": 1,
                            "maturity": 1,
                            "projected_center_in_image": False,
                            "depth_visible": False,
                            "containing_detection_indices": [],
                        }
                    ],
                    "detections": [],
                }
            ]
        )
        self.assertEqual(result["visible_ripe_truth_count"], 0)
        self.assertEqual(result["all_ripe_localized_scene_count"], 0)
        self.assertFalse(result["scenes"][0]["all_ripe_localized"])


if __name__ == "__main__":
    unittest.main()
