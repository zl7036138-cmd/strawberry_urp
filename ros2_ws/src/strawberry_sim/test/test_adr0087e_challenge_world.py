from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MATERIALIZER_PATH = (
    REPOSITORY_ROOT / "scripts" / "materialize_adr0087e_challenge_world.py"
)
MODULE_SPEC = spec_from_file_location("adr0087e_challenge_world", MATERIALIZER_PATH)
assert MODULE_SPEC is not None and MODULE_SPEC.loader is not None
MATERIALIZER = module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(MATERIALIZER)


class Adr0087EChallengeWorldTests(unittest.TestCase):
    def test_materialized_world_contains_one_physical_parity_box(self):
        base_world = (
            REPOSITORY_ROOT
            / "ros2_ws"
            / "src"
            / "strawberry_sim"
            / "worlds"
            / "strawberry_orchard.sdf"
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "challenge.sdf"
            receipt = MATERIALIZER.materialize_world(
                base_world,
                output,
                "0.348 0.153 0.730 0.025 0.025 0.050",
            )
            self.assertEqual(
                receipt["model_name"],
                "development_candidate_challenge_obstacle",
            )
            self.assertEqual(
                receipt["obstacle_spec"],
                [0.348, 0.153, 0.73, 0.025, 0.025, 0.05],
            )
            root = ET.parse(output).getroot()
            models = root.findall(
                "./world/model[@name='development_candidate_challenge_obstacle']"
            )
            self.assertEqual(len(models), 1)
            model = models[0]
            self.assertEqual(
                model.findtext("static"), "true"
            )
            self.assertEqual(
                tuple(float(value) for value in model.findtext("pose").split()[:3]),
                (0.348, 0.153, 0.73),
            )
            self.assertEqual(
                tuple(
                    float(value)
                    for value in model.findtext(
                        "./link/collision/geometry/box/size"
                    ).split()
                ),
                (0.025, 0.025, 0.05),
            )
            with self.assertRaises(FileExistsError):
                MATERIALIZER.materialize_world(
                    base_world,
                    output,
                    "0.348 0.153 0.730 0.025 0.025 0.050",
                )


if __name__ == "__main__":
    unittest.main()
