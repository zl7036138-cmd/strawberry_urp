from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DartAuditContractTests(unittest.TestCase):
    def test_original_called_once_with_unchanged_reset_flag(self):
        text = (ROOT / 'StepAudit.cc').read_text()
        self.assertEqual(text.count('call(this, resetCommand);'), 1)
        self.assertLess(text.index('record("BEFORE_DART_STEP", this, resetCommand);'),
                        text.index('call(this, resetCommand);'))
        self.assertGreater(text.index('record("AFTER_DART_STEP", this, resetCommand);'),
                           text.index('call(this, resetCommand);'))

    def test_no_physics_mutations_and_output_is_exclusive(self):
        text = (ROOT / 'StepAudit.cc').read_text()
        for setter in ('setCommand(', 'setPosition(', 'setVelocity(', 'setForce(', 'setActuatorType('):
            self.assertNotIn(setter, text)
        self.assertIn('STRAWBERRY_DART_AUDIT_OUTPUT', text)
        self.assertIn('std::fopen(path, "wx")', text)
        for getter in ('getCommand(0)', 'getConstraintImpulse(0)', 'getIndexInSkeleton(0)'):
            self.assertIn(getter, text)


if __name__ == '__main__': unittest.main()
