from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.truth_isolation_audit import (  # noqa: E402
    audit_truth_subscriptions,
    build_audit_payload,
    normalized_node_name,
)


class TruthIsolationAuditTests(unittest.TestCase):
    def test_node_names_are_normalized(self):
        self.assertEqual(normalized_node_name("node", "/scope"), "/scope/node")
        self.assertEqual(normalized_node_name("/node"), "/node")

    def test_generalized_control_truth_subscription_fails(self):
        violations = audit_truth_subscriptions(
            {
                "/strawberry_base_localization": [
                    "/strawberry/ground_truth/poses"
                ],
                "/attachment_manager": [
                    "/strawberry/ground_truth/fruit_1/pose"
                ],
            }
        )

        self.assertEqual(
            violations,
            (
                {
                    "node": "/strawberry_base_localization",
                    "topic": "/strawberry/ground_truth/poses",
                },
            ),
        )

    def test_only_adapter_truth_subscriptions_pass(self):
        payload = build_audit_payload(
            {
                "/attachment_manager": [
                    "/strawberry/ground_truth/fruit_1/pose"
                ],
                "/strawberry_base_localization": [
                    "/camera/base/depth/image_raw"
                ],
                "/strawberry_pick_and_place": [
                    "/strawberry/tracked_targets"
                ],
            },
            allowed_nodes=("/attachment_manager",),
            required_nodes=(
                "/strawberry_base_localization",
                "/strawberry_pick_and_place",
            ),
        )

        self.assertTrue(payload["overall_pass"])
        self.assertEqual(payload["violations"], [])

    def test_missing_required_control_node_fails_closed(self):
        payload = build_audit_payload(
            {"/attachment_manager": []},
            allowed_nodes=("/attachment_manager",),
            required_nodes=("/strawberry_base_localization",),
        )

        self.assertFalse(payload["overall_pass"])
        self.assertEqual(
            payload["missing_required_nodes"], ["/strawberry_base_localization"]
        )


if __name__ == "__main__":
    unittest.main()
