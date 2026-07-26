from pathlib import Path
import re
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BUILD_SCRIPT = REPOSITORY_ROOT / "scripts" / "build_and_test.sh"


class ColconInterpreterContractTests(unittest.TestCase):
    def test_build_uses_perception_venv_and_tests_use_apt_colcon(
        self,
    ) -> None:
        text = BUILD_SCRIPT.read_text(encoding="utf-8")

        self.assertIn('colcon_python="${VIRTUAL_ENV}/bin/python"', text)
        self.assertIn('colcon_executable="$(command -v colcon)"', text)
        self.assertEqual(
            1,
            text.count('"${colcon_python}" "${colcon_executable}"'),
            "only build must run under the venv Python to fix installed shebangs",
        )
        self.assertIn('"${colcon_executable}" \\\n  --log-base "${artifact_root}/log" test', text)
        self.assertIn('"${colcon_executable}" test-result', text)
        self.assertIn(
            'expected_shebang="#!${colcon_python}"',
            text,
        )
        self.assertIn(
            'actual_shebang < "${perception_entry}"',
            text,
        )

        bare_invocations = [
            line
            for line in text.splitlines()
            if re.match(r"^\s*colcon(?:\s|$)", line)
        ]
        self.assertEqual([], bare_invocations)


if __name__ == "__main__":
    unittest.main()
