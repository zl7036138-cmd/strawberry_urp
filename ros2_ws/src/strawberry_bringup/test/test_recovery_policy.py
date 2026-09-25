import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.recovery_policy import (  # noqa: E402
    RecoveryDisposition,
    RecoveryStep,
    recovery_step_for_disposition,
)


class RecoveryPolicyTests(unittest.TestCase):
    def test_wire_values_match_fail_closed_action_contract(self):
        self.assertEqual(int(RecoveryDisposition.MOTION_WITHHELD), 0)
        self.assertEqual(int(RecoveryDisposition.AT_HOME), 1)
        self.assertEqual(int(RecoveryDisposition.HOME_REQUIRED), 2)

    def test_each_known_disposition_has_one_bounded_step(self):
        self.assertEqual(
            recovery_step_for_disposition(RecoveryDisposition.MOTION_WITHHELD),
            RecoveryStep.WITHHOLD,
        )
        self.assertEqual(
            recovery_step_for_disposition(RecoveryDisposition.AT_HOME),
            RecoveryStep.ALREADY_HOME,
        )
        self.assertEqual(
            recovery_step_for_disposition(RecoveryDisposition.HOME_REQUIRED),
            RecoveryStep.REQUEST_HOME,
        )

    def test_unknown_or_malformed_values_fail_closed(self):
        for value in (-1, 3, 255, None, "unknown"):
            with self.subTest(value=value):
                self.assertEqual(
                    recovery_step_for_disposition(value), RecoveryStep.WITHHOLD
                )

    def test_orchestrator_uses_typed_result_and_withheld_terminal(self):
        source = (
            PACKAGE_ROOT / "strawberry_bringup" / "harvest_orchestrator.py"
        ).read_text(encoding="utf-8")
        self.assertIn("int(result.recovery_disposition)", source)
        self.assertIn("RecoveryStep.WITHHOLD", source)
        self.assertIn('"RECOVERY_MOTION_WITHHELD"', source)
        self.assertIn("RecoveryStep.ALREADY_HOME", source)


if __name__ == "__main__":
    unittest.main()
