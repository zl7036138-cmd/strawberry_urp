import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_sim.runtime_health import (  # noqa: E402
    TopicObservation,
    measured_rates,
)


class RuntimeHealthTests(unittest.TestCase):
    def test_camera_runtime_health_selection_is_parameterized(self):
        source = (
            PACKAGE_ROOT / "strawberry_sim" / "runtime_health.py"
        ).read_text(encoding="utf-8")
        self.assertIn('parser.add_argument("--camera-prefix", default="/camera")', source)
        self.assertIn(
            '"--camera-frame", default="strawberry_camera_optical_frame"', source
        )
        self.assertIn('parser.add_argument("--expected-width", type=int, default=640)', source)
        self.assertIn('parser.add_argument("--expected-height", type=int, default=480)', source)
        self.assertIn('f"{camera_prefix}/color/image_raw"', source)
        self.assertIn('f"{camera_prefix}/depth/image_raw"', source)
        self.assertIn('f"{camera_prefix}/camera_info"', source)

    def test_observation_records_first_and_latest_receipt(self):
        observation = TopicObservation()
        observation.record(10.0)
        observation.record(11.5)
        self.assertEqual(observation.count, 2)
        self.assertEqual(observation.first_wall_sec, 10.0)
        self.assertEqual(observation.last_wall_sec, 11.5)

    def test_rates_exclude_startup_baseline(self):
        observations = {"image": TopicObservation(count=25)}
        self.assertEqual(measured_rates(observations, {"image": 5}, 4.0), {"image": 5.0})

    def test_rates_require_positive_elapsed_time(self):
        with self.assertRaises(ValueError):
            measured_rates({"clock": TopicObservation()}, {}, 0.0)


if __name__ == "__main__":
    unittest.main()
