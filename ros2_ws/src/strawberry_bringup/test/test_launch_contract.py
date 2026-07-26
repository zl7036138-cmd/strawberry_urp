import ast
import pathlib
import re
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


def _launch_tree() -> ast.Module:
    return ast.parse(
        (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
    )


def _call_keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next(
        (keyword.value for keyword in call.keywords if keyword.arg == name),
        None,
    )


class LaunchContractTests(unittest.TestCase):
    def test_system_launch_composes_all_pipeline_packages(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        for package in (
            "strawberry_sim",
            "strawberry_perception",
            "strawberry_localization",
            "strawberry_manipulation",
            "strawberry_bringup",
        ):
            expected = (
                'get_package_share_directory("strawberry_sim")'
                if package == "strawberry_sim"
                else f'package="{package}"'
            )
            self.assertIn(expected, text)

    def test_unverified_motion_paths_default_to_fail_closed(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"start_manipulation",\s*default_value="false"',
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"enable_attachment",\s*default_value="false"',
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"enable_pose_control",\s*default_value="false"',
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"start_oracle_provider",\s*default_value="false"',
        )

    def test_pose_control_is_forwarded_only_through_explicit_launch_argument(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('enable_pose_control = LaunchConfiguration("enable_pose_control")', text)
        self.assertIn('"enable_pose_control": enable_pose_control', text)

    def test_materialized_world_is_forwarded_only_by_explicit_argument(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"world_file",\s*default_value=""',
        )
        self.assertIn('world_file = LaunchConfiguration("world_file")', text)
        self.assertIn('"world_file": world_file', text)

    def test_camera_mount_defaults_fixed_and_is_forwarded_explicitly(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"camera_mount",\s*default_value="fixed"',
        )
        self.assertIn('camera_mount = LaunchConfiguration("camera_mount")', text)
        self.assertIn('"camera_mount": camera_mount', text)
        self.assertIn('{"use_sim_time": True, "camera_mount": camera_mount}', text)
        self.assertIn('choices=["fixed", "wrist", "dual"]', text)

    def test_perception_camera_topics_are_explicit_and_keep_legacy_defaults(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        defaults = {
            "perception_image_topic": "/camera/color/image_raw",
            "localization_depth_topic": "/camera/depth/image_raw",
            "localization_camera_info_topic": "/camera/camera_info",
        }
        for argument, default in defaults.items():
            self.assertRegex(
                text,
                rf'DeclareLaunchArgument\(\s*"{argument}",\s*'
                rf'default_value="{default}"',
            )
            self.assertRegex(
                text,
                rf"{argument}\s*=\s*LaunchConfiguration\(\s*"
                rf'"{argument}"\s*\)',
            )
        self.assertIn('"image_topic": perception_image_topic', text)
        self.assertIn('"depth_topic": localization_depth_topic', text)
        self.assertIn('"camera_info_topic": localization_camera_info_topic', text)

    def test_localization_attention_roi_defaults_disabled_and_is_typed(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        for bound in ("min_x", "min_y", "max_x", "max_y"):
            argument = f"localization_selection_roi_{bound}_px"
            self.assertRegex(
                text,
                rf'DeclareLaunchArgument\(\s*"{argument}",\s*default_value="-1"',
            )
            self.assertIn(f'"selection_roi_{bound}_px": ParameterValue(', text)
        self.assertIn("**selection_roi_parameters", text)

    def test_oracle_control_and_shadow_topics_are_explicitly_separate(self) -> None:
        text = (PACKAGE_ROOT / "launch" / "system.launch.py").read_text(
            encoding="utf-8"
        )
        for argument, default in (
            ("oracle_target_topic", "/strawberry/oracle/target_pose"),
            ("control_target_topic", "/strawberry/oracle/target_pose"),
            ("perception_target_topic", "/strawberry/shadow/target_pose"),
            ("shadow_target_topic", "/strawberry/shadow/target_pose"),
            ("shadow_detections_topic", "/strawberry/shadow/detections"),
        ):
            self.assertRegex(
                text,
                rf'DeclareLaunchArgument\(\s*"{argument}",\s*default_value="{re.escape(default)}"',
            )
        self.assertIn('executable="oracle_target_provider"', text)
        self.assertIn('"control_target_topic": control_target_topic', text)
        self.assertIn('"shadow_target_topic": shadow_target_topic', text)

    def test_trial_service_checks_action_readiness(self) -> None:
        text = (
            PACKAGE_ROOT / "strawberry_bringup" / "orchestrator.py"
        ).read_text(encoding="utf-8")
        self.assertIn("if not self._action.server_is_ready():", text)
        self.assertIn('response.message = "pick action is not ready"', text)

    def test_confidence_threshold_is_one_typed_shared_launch_argument(self) -> None:
        tree = _launch_tree()

        declarations = [
            call
            for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "DeclareLaunchArgument"
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == "confidence_threshold"
        ]
        self.assertEqual(len(declarations), 1)
        default = _call_keyword(declarations[0], "default_value")
        self.assertIsInstance(default, ast.Constant)
        self.assertIsInstance(default.value, str)
        self.assertEqual(default.value, "0.60")

        launch_configuration_assignments = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            for node in node.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "confidence_threshold"
                for target in node.targets
            )
        ]
        self.assertEqual(len(launch_configuration_assignments), 1)
        launch_configuration = launch_configuration_assignments[0].value
        self.assertIsInstance(launch_configuration, ast.Call)
        self.assertIsInstance(launch_configuration.func, ast.Name)
        self.assertEqual(launch_configuration.func.id, "LaunchConfiguration")
        self.assertEqual(launch_configuration.args[0].value, "confidence_threshold")

        typed_parameter_assignments = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            for node in node.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "confidence_threshold_parameter"
                for target in node.targets
            )
        ]
        self.assertEqual(len(typed_parameter_assignments), 1)
        typed_parameter = typed_parameter_assignments[0].value
        self.assertIsInstance(typed_parameter, ast.Call)
        self.assertIsInstance(typed_parameter.func, ast.Name)
        self.assertEqual(typed_parameter.func.id, "ParameterValue")
        self.assertIsInstance(typed_parameter.args[0], ast.Name)
        self.assertEqual(typed_parameter.args[0].id, "confidence_threshold")
        value_type = _call_keyword(typed_parameter, "value_type")
        self.assertIsInstance(value_type, ast.Name)
        self.assertEqual(value_type.id, "float")

        pipeline_nodes = {}
        for call in ast.walk(tree):
            if not (
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Name)
                and call.func.id == "Node"
            ):
                continue
            package = _call_keyword(call, "package")
            if isinstance(package, ast.Constant) and package.value in {
                "strawberry_perception",
                "strawberry_localization",
            }:
                pipeline_nodes[package.value] = call

        self.assertEqual(
            set(pipeline_nodes),
            {"strawberry_perception", "strawberry_localization"},
        )
        for package, node in pipeline_nodes.items():
            condition = _call_keyword(node, "condition")
            self.assertIsInstance(condition, ast.Call, package)
            self.assertIsInstance(condition.func, ast.Name, package)
            self.assertEqual(condition.func.id, "IfCondition", package)
            self.assertIsInstance(condition.args[0], ast.Name, package)
            self.assertEqual(condition.args[0].id, "start_perception", package)

            parameters = _call_keyword(node, "parameters")
            self.assertIsInstance(parameters, ast.List, package)
            overrides = [
                value
                for item in parameters.elts
                if isinstance(item, ast.Dict)
                for key, value in zip(item.keys, item.values)
                if isinstance(key, ast.Constant)
                and key.value == "confidence_threshold"
            ]
            self.assertEqual(len(overrides), 1, package)
            self.assertIsInstance(overrides[0], ast.Name, package)
            self.assertEqual(
                overrides[0].id,
                "confidence_threshold_parameter",
                package,
            )

    def test_package_configs_keep_numeric_point_six_fallback(self) -> None:
        config_paths = (
            PACKAGE_ROOT.parent
            / "strawberry_perception"
            / "config"
            / "perception.yaml",
            PACKAGE_ROOT.parent
            / "strawberry_localization"
            / "config"
            / "localization.yaml",
        )
        for path in config_paths:
            text = path.read_text(encoding="utf-8")
            match = re.search(
                r"^\s+confidence_threshold:\s+([^\s#]+)",
                text,
                flags=re.MULTILINE,
            )
            self.assertIsNotNone(match, path)
            token = match.group(1)
            self.assertEqual(token, "0.60", path)
            self.assertEqual(float(token), 0.60, path)


if __name__ == "__main__":
    unittest.main()
