import contextlib
import importlib.util
import io
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SIM_PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_sim"
sys.path.insert(0, str(SIM_PACKAGE_ROOT))

VALIDATOR_PATH = (
    REPOSITORY_ROOT
    / "scripts"
    / "validate_rendered_occlusion_localization_matrix.py"
)
MANIFEST_PATH = (
    REPOSITORY_ROOT
    / "config"
    / "rendered_occlusion_localization_matrix_v1.json"
)
RUNNER_PATH = (
    REPOSITORY_ROOT
    / "scripts"
    / "run_rendered_occlusion_localization_matrix.sh"
)

SPEC = importlib.util.spec_from_file_location(
    "rendered_occlusion_matrix_validator", VALIDATOR_PATH
)
VALIDATOR = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(VALIDATOR)


class RenderedOcclusionMatrixTests(unittest.TestCase):
    def test_frozen_manifest_loads_and_contains_fifteen_scenarios(self) -> None:
        manifest = VALIDATOR.load_manifest(MANIFEST_PATH)

        self.assertEqual(len(manifest["positions"]), 5)
        self.assertEqual(len(manifest["conditions"]), 3)
        self.assertEqual(manifest["runtime"]["scenario_count"], 15)
        self.assertEqual(manifest["execution"]["maximum_claims"], 1)
        self.assertFalse(manifest["execution"]["retry_authorized"])

    def test_emitted_matrix_has_fifteen_unique_rows(self) -> None:
        previous_argv = sys.argv
        output = io.StringIO()
        try:
            sys.argv = [
                str(VALIDATOR_PATH),
                "--manifest",
                str(MANIFEST_PATH),
                "--emit-tsv",
            ]
            with contextlib.redirect_stdout(output):
                status = VALIDATOR.main()
        finally:
            sys.argv = previous_argv

        rows = output.getvalue().strip().splitlines()
        self.assertEqual(status, 0)
        self.assertEqual(len(rows), 15)
        self.assertEqual(len({row.split("\t")[1] for row in rows}), 15)

    def test_runner_preserves_no_motion_and_no_perception_boundary(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")

        self.assertIn("camera_mount:=fixed", source)
        self.assertIn("start_perception:=false", source)
        self.assertIn("start_manipulation:=false", source)
        self.assertIn("start_orchestrator:=false", source)
        self.assertIn("enable_attachment:=false", source)
        self.assertIn("ground_truth_association_enabled:=false", source)
        self.assertIn("localization_rendered_center_v1.yaml", source)
        self.assertIn("localization_rendered_geometry_layer_v1.yaml", source)
        self.assertIn("__node:=rendered_center_localization", source)
        self.assertIn("__node:=rendered_geometry_localization", source)
        self.assertIn('--bbox-padding "${oracle_bbox_padding}"', source)
        self.assertIn("Installed localization core is stale", source)


if __name__ == "__main__":
    unittest.main()
