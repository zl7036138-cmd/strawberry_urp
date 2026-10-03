import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    CameraIntrinsics,
    LocalizationError,
)
from strawberry_localization.gate_core import (  # noqa: E402
    benchmark_positions,
    linear_quantile,
    sphere_projection_bbox,
    summarize_gate,
)
from strawberry_localization.localization_gate import (  # noqa: E402
    SIMULATION_SHARE_PROVENANCE_PATHS,
    _file_fingerprint,
)
from strawberry_localization.v2_gate_contract import (  # noqa: E402
    load_contract as load_v2_contract,
    position_grid as v2_position_grid,
)


REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
INITIAL_V2_CONTRACT = (
    REPOSITORY_ROOT
    / "config"
    / "blender_v2_localization_accuracy_100_v1.json"
)
POST_WINDING_V2_CONTRACT = (
    REPOSITORY_ROOT
    / "config"
    / "blender_v2_localization_accuracy_100_post_winding_v1.json"
)


class LocalizationGateCoreTests(unittest.TestCase):
    def test_provenance_covers_result_determining_simulation_assets(self) -> None:
        expected = {
            "launch/sim.launch.py",
            "worlds/strawberry_orchard.sdf",
            "models/rgbd_camera/model.sdf",
            "models/strawberry_ripe/model.sdf",
            "models/strawberry_unripe/model.sdf",
            "config/bridge.yaml",
            "config/scene.yaml",
            "config/sim_nodes.yaml",
        }
        self.assertTrue(expected.issubset(set(SIMULATION_SHARE_PROVENANCE_PATHS)))

    def test_file_fingerprint_records_size_and_sha256(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = pathlib.Path(temporary_directory) / "evidence.txt"
            path.write_bytes(b"T40\n")

            fingerprint = _file_fingerprint(path)

        self.assertEqual(fingerprint["name"], "evidence.txt")
        self.assertEqual(fingerprint["size_bytes"], 4)
        self.assertEqual(
            fingerprint["sha256"],
            "d086ba5f35fbe5f48790de01a0fc36f47272d0231ca38245f658a4baba33b4ec",
        )

    def test_runtime_gate_is_packaged_with_simulator_offset(self) -> None:
        setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
        config_text = (PACKAGE_ROOT / "config" / "localization.yaml").read_text(
            encoding="utf-8"
        )
        launch_text = (
            PACKAGE_ROOT / "launch" / "localization_gate.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("localization_gate =", setup_text)
        self.assertIn("surface_to_center_offset_m: 0.035", config_text)
        self.assertIn('"enable_pose_control": "true"', launch_text)

    def test_blender_v2_config_has_separate_26_mm_offset(self) -> None:
        v1_config = (
            PACKAGE_ROOT / "config" / "localization.yaml"
        ).read_text(encoding="utf-8")
        v2_config = (
            PACKAGE_ROOT / "config" / "localization_blender_v2.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("surface_to_center_offset_m: 0.035", v1_config)
        self.assertIn("surface_to_center_offset_m: 0.026", v2_config)

    def test_renamed_dual_camera_nodes_receive_v2_offset_explicitly(self) -> None:
        runner = (
            REPOSITORY_ROOT / "scripts" / "run_dual_sequential_observation.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("__node:=strawberry_base_overview_localization", runner)
        self.assertIn("__node:=strawberry_wrist_attention_localization", runner)
        self.assertEqual(
            runner.count("-p surface_to_center_offset_m:=0.026"),
            2,
        )

    def test_default_grid_contains_one_hundred_distinct_positions(self) -> None:
        positions = benchmark_positions()
        self.assertEqual(len(positions), 100)
        self.assertEqual(len(set(positions)), 100)

    def test_blender_v2_gate_freezes_separate_camera_clear_grid(self) -> None:
        contract = load_v2_contract(
            POST_WINDING_V2_CONTRACT,
            REPOSITORY_ROOT,
        )
        positions = v2_position_grid(contract)
        self.assertEqual(len(positions), 100)
        self.assertEqual(len(set(positions)), 100)
        self.assertEqual(positions[0], (0.44, -0.12, 0.51))
        self.assertEqual(positions[-1], (0.56, 0.06, 0.57))
        self.assertEqual(contract["target"]["fruit_radius_m"], 0.026)
        self.assertFalse(contract["safety"]["robot_motion_authorized"])
        self.assertTrue(
            contract["safety"]["simulated_fruit_pose_motion_authorized"]
        )

    def test_consumed_pre_repair_contract_rejects_repaired_mesh(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "target_visual_mesh binding changed",
        ):
            load_v2_contract(INITIAL_V2_CONTRACT, REPOSITORY_ROOT)

    def test_blender_v2_gate_launch_and_runner_remain_fail_closed(self) -> None:
        launch_text = (
            PACKAGE_ROOT
            / "launch"
            / "blender_v2_localization_gate.launch.py"
        ).read_text(encoding="utf-8")
        runner_text = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_localization_gate.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("localization_blender_v2.yaml", launch_text)
        self.assertIn('"enable_attachment": "false"', launch_text)
        self.assertIn('"enable_pose_control": "true"', launch_text)
        self.assertIn('"camera_mount": "fixed"', launch_text)
        self.assertIn("--gate-contract", runner_text)
        self.assertNotIn("start_manipulation:=true", runner_text)
        self.assertNotIn("enable_attachment:=true", runner_text)

    def test_sphere_projection_contains_principal_point(self) -> None:
        box = sphere_projection_bbox(
            (0.0, 0.0, 1.0),
            CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
            sphere_radius_m=0.035,
            image_width=640,
            image_height=480,
        )
        self.assertLessEqual(box.x, 320)
        self.assertGreaterEqual(box.x + box.width, 320)
        self.assertLessEqual(box.y, 240)
        self.assertGreaterEqual(box.y + box.height, 240)

    def test_projection_rejects_a_point_outside_the_image(self) -> None:
        with self.assertRaises(LocalizationError):
            sphere_projection_bbox(
                (2.0, 0.0, 1.0),
                CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
                sphere_radius_m=0.035,
                image_width=640,
                image_height=480,
            )

    def test_quantile_uses_linear_interpolation(self) -> None:
        self.assertAlmostEqual(linear_quantile((0.0, 10.0), 0.95), 9.5)

    def test_gate_requires_all_one_hundred_measurements(self) -> None:
        passed = summarize_gate([10.0] * 100, requested_positions=100)
        self.assertTrue(passed["passed"])
        missing = summarize_gate([10.0] * 99, requested_positions=100)
        self.assertFalse(missing["passed"])
        high_tail = summarize_gate(
            [10.0] * 94 + [35.0] * 6,
            requested_positions=100,
        )
        self.assertFalse(high_tail["passed"])


if __name__ == "__main__":
    unittest.main()
