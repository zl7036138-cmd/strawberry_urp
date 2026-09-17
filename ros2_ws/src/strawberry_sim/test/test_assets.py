import pathlib
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

from strawberry_sim.core import load_scene_config
from strawberry_sim.obj_winding import winding_stats


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class SimulationAssetTests(unittest.TestCase):
    def test_package_data_excludes_transient_python_bytecode(self):
        setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
        self.assertIn('dirname != "__pycache__"', setup_text)
        self.assertIn('filename.endswith((".pyc", ".pyo"))', setup_text)

    def test_camera_tf_uses_ros_optical_axis_conversion(self):
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"--child-frame-id", "strawberry_camera_link"', launch_text)
        self.assertIn('name="camera_optical_static_tf"', launch_text)
        self.assertIn('"--roll", "-1.5707963268"', launch_text)
        self.assertIn('"--yaw", "-1.5707963268"', launch_text)

    def test_pose_control_service_is_explicitly_opt_in(self):
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            launch_text,
            r'DeclareLaunchArgument\(\s*"enable_pose_control",\s*default_value="false"',
        )
        self.assertIn("ros_gz_interfaces/srv/SetEntityPose", launch_text)

    def test_materialized_world_override_is_explicit_and_fails_closed(self):
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            launch_text,
            r'DeclareLaunchArgument\(\s*"world_file",\s*default_value=""',
        )
        self.assertIn('LaunchConfiguration("world_file").perform(context)', launch_text)
        self.assertIn("if not os.path.isfile(world_file):", launch_text)

    def test_initial_joint_override_is_explicit_and_v2_default_is_preserved(self):
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            launch_text,
            r'DeclareLaunchArgument\(\s*"initial_positions_file",\s*default_value=""',
        )
        self.assertIn(
            'LaunchConfiguration("initial_positions_file").perform(context)',
            launch_text,
        )
        field_launch = (
            PACKAGE_ROOT / "launch" / "field_v3.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("panda_initial_positions_field_v3.yaml", field_launch)
        field_positions = yaml.safe_load(
            (
                PACKAGE_ROOT
                / "config"
                / "panda_initial_positions_field_v3.yaml"
            ).read_text(encoding="utf-8")
        )["initial_positions"]
        self.assertEqual(len(field_positions), 7)
        self.assertAlmostEqual(field_positions["panda_joint1"], 1.310378291)

    def test_simulation_seed_is_explicit_opt_in_and_reaches_gazebo(self):
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            launch_text,
            r'DeclareLaunchArgument\(\s*"simulation_seed",\s*default_value=""',
        )
        self.assertIn('gz_command.extend(["--seed", str(simulation_seed)])', launch_text)
        self.assertIn("if not 0 <= simulation_seed <= 0xFFFFFFFF:", launch_text)

    def test_every_xml_asset_is_well_formed(self):
        files = (
            list(PACKAGE_ROOT.rglob("*.sdf"))
            + list(PACKAGE_ROOT.rglob("*.config"))
            + list(PACKAGE_ROOT.rglob("*.xacro"))
        )
        self.assertGreaterEqual(len(files), 9)
        for path in files:
            with self.subTest(path=path):
                ET.parse(path)

    def test_world_contains_fixed_scene_contract(self):
        root = ET.parse(PACKAGE_ROOT / "worlds" / "strawberry_orchard.sdf").getroot()
        names = [element.findtext("name") for element in root.findall(".//include")]
        self.assertIn("strawberry_rgbd_camera", names)
        self.assertIn("collection_bin", names)
        self.assertIn("strawberry_plant", names)
        self.assertEqual(sum(name and name.startswith("strawberry_") and name[-1:].isdigit() for name in names), 3)

    def test_blender_visual_assets_are_the_canonical_scene(self):
        world = ET.parse(
            PACKAGE_ROOT / "worlds" / "strawberry_orchard.sdf"
        ).getroot()
        plant = world.find("./world/include[name='strawberry_plant']")
        self.assertEqual(plant.findtext("uri"), "model://strawberry_plant_v2")
        for model_name in ("strawberry_ripe", "strawberry_unripe"):
            model = ET.parse(
                PACKAGE_ROOT / "models" / model_name / "model.sdf"
            ).getroot()
            mesh_uri = model.findtext("./model/link/visual/geometry/mesh/uri")
            self.assertIn(f"model://{model_name}/meshes/", mesh_uri)
            mesh_path = (
                PACKAGE_ROOT
                / "models"
                / model_name
                / "meshes"
                / pathlib.Path(mesh_uri).name
            )
            self.assertTrue(mesh_path.is_file())
            self.assertGreater(mesh_path.stat().st_size, 100_000)

    def test_field_v3_preserves_instancing_and_simple_collisions(self):
        model_root = PACKAGE_ROOT / "models" / "strawberry_field_v3"
        manifest = yaml.safe_load(
            (model_root / "export_manifest.json").read_text(encoding="utf-8")
        )["field"]
        self.assertEqual(manifest["source_plant_instances"], 103)
        self.assertEqual(manifest["exported_plant_instances"], 101)
        self.assertEqual(manifest["unique_plant_variants"], 24)
        self.assertEqual(
            manifest["omitted_source_plants"],
            ["草莓植株_010", "草莓植株_026"],
        )
        self.assertEqual(manifest["plant_decimation_ratio"], 0.45)
        self.assertEqual(
            manifest["collisions"][1]["adjustments"],
            {
                "minimum_x_m": 0.2,
                "reason": "fixed Panda pedestal clearance",
            },
        )

        variants = manifest["variants"]
        self.assertEqual(len(variants), 24)
        self.assertTrue(all(variant["triangles"] > 0 for variant in variants))
        for variant in variants:
            self.assertTrue(
                (model_root / "meshes" / variant["mesh_file"]).is_file()
            )

        model = ET.parse(model_root / "model.sdf").getroot()
        self.assertEqual(model.find("./model").attrib["name"], "strawberry_field_v3")
        self.assertEqual(model.findtext("./model/static"), "true")
        collisions = model.findall("./model/link/collision")
        visuals = model.findall("./model/link/visual")
        self.assertEqual(len(collisions), 4)
        ridge_pose = tuple(
            float(value) for value in collisions[1].findtext("pose").split()
        )
        ridge_size = tuple(
            float(value)
            for value in collisions[1].findtext("./geometry/box/size").split()
        )
        self.assertAlmostEqual(
            ridge_pose[0] - ridge_size[0] / 2.0,
            0.2,
            places=7,
        )
        self.assertEqual(len(visuals), 102)
        self.assertTrue(
            all(collision.find("./geometry/box") is not None for collision in collisions)
        )
        mesh_uris = {
            visual.findtext("./geometry/mesh/uri")
            for visual in visuals
        }
        self.assertEqual(len(mesh_uris), 25)
        self.assertTrue(
            all(uri.startswith("model://strawberry_field_v3/meshes/") for uri in mesh_uris)
        )

    def test_field_v3_world_is_opt_in_and_matches_its_manifest(self):
        world = ET.parse(
            PACKAGE_ROOT / "worlds" / "strawberry_field_v3.sdf"
        ).getroot()
        includes = {
            include.findtext("name"): include
            for include in world.findall("./world/include")
        }
        self.assertEqual(
            includes["strawberry_field_environment"].findtext("uri"),
            "model://strawberry_field_v3",
        )
        self.assertIn("strawberry_rgbd_camera", includes)
        self.assertIn("strawberry_plant", includes)
        self.assertEqual(
            tuple(
                float(value)
                for value in includes["collection_bin"].findtext("pose").split()
            ),
            (-0.90, 0.80, 0.0, 0.0, 0.0, 0.0),
        )

        scene = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "scene_field_v3.yaml").read_text(
                encoding="utf-8"
            )
        )
        qualification = scene["qualification"]
        self.assertTrue(qualification["motion_authorized"])
        self.assertEqual(
            qualification["profile_id"],
            "field_v3_perception_pick_repeat_v1",
        )
        self.assertEqual(
            qualification["authorization_scope"],
            "exact_bounded_perception_pick_runner",
        )
        self.assertFalse(qualification["formal_acceptance"])
        self.assertEqual(scene["camera"]["default_mount"], "dual")
        self.assertEqual(
            scene["planning_scene"]["static_collision_profile"],
            "field_v3",
        )
        self.assertEqual(scene["bin"]["place_pose_m"], [-0.45, 0.25, 0.45])
        self.assertAlmostEqual(
            scene["bin"]["place_transit_clearance_m"], 0.12
        )
        place_x, place_y, place_z = scene["bin"]["place_pose_m"]
        bounds = scene["bin"]["interior_bounds_m"]
        self.assertGreaterEqual(place_x, bounds["min_x"])
        self.assertLessEqual(place_x, bounds["max_x"])
        self.assertGreaterEqual(place_y, bounds["min_y"])
        self.assertLessEqual(place_y, bounds["max_y"])
        self.assertGreaterEqual(place_z, bounds["min_z"])
        self.assertLessEqual(place_z, bounds["max_z"])
        self.assertEqual(
            scene["camera"]["wrist_target_1_selection_roi_xyxy_px"],
            [320, 240, 640, 480],
        )
        self.assertEqual(scene["field"]["background_plant_count"], 101)
        world_fruit_poses = {
            name: tuple(
                float(value)
                for value in includes[name].findtext("pose").split()[:3]
            )
            for name in ("strawberry_1", "strawberry_2", "strawberry_3")
        }
        manifest_fruit_poses = {
            fruit["model_name"]: tuple(fruit["initial_pose_m"])
            for fruit in scene["fruits"]
        }
        self.assertEqual(world_fruit_poses, manifest_fruit_poses)

        launch_text = (
            PACKAGE_ROOT / "launch" / "field_v3.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('default_value="dual"', launch_text)
        self.assertIn('"enable_attachment": "false"', launch_text)
        self.assertIn('"enable_pose_control": "false"', launch_text)
        self.assertNotIn("strawberry_field_v3.sdf", (
            PACKAGE_ROOT / "launch" / "sim.launch.py"
        ).read_text(encoding="utf-8"))

    def test_blender_v2_fruit_body_faces_have_outward_winding(self):
        for model_name, mesh_name in (
            ("strawberry_ripe", "strawberry_ripe_visual_v2.obj"),
            ("strawberry_unripe", "strawberry_unripe_visual_v2.obj"),
        ):
            stats = winding_stats(
                PACKAGE_ROOT / "models" / model_name / "meshes" / mesh_name,
                "strawberry_ripe_body_v5",
            )
            self.assertEqual(stats.triangle_count, 11796)
            self.assertEqual(stats.outward_triangles, 11796)
            self.assertEqual(stats.inward_triangles, 0)
            self.assertEqual(stats.degenerate_triangles, 0)

    def test_archived_tabletop_world_uses_archived_sphere_assets(self):
        world = ET.parse(
            PACKAGE_ROOT / "worlds" / "strawberry_tabletop_benchmark_v1.sdf"
        ).getroot()
        fruit_uris = {
            include.findtext("uri")
            for include in world.findall("./world/include")
            if include.findtext("name", "").startswith("strawberry_")
            and include.findtext("name", "")[-1:].isdigit()
        }
        self.assertEqual(
            fruit_uris,
            {
                "model://strawberry_ripe_tabletop_v1",
                "model://strawberry_unripe_tabletop_v1",
            },
        )
        for asset in ("strawberry_ripe_tabletop_v1", "strawberry_unripe_tabletop_v1"):
            self.assertTrue((PACKAGE_ROOT / "models" / asset / "model.sdf").is_file())

    def test_rgbd_camera_resolution_and_topics(self):
        root = ET.parse(PACKAGE_ROOT / "models" / "rgbd_camera" / "model.sdf").getroot()
        sensor = root.find(".//sensor")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib["type"], "rgbd_camera")
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "480")
        self.assertEqual(sensor.findtext("topic"), "/camera")

    def test_robot_camera_mounts_are_opt_in_reusable_and_topic_isolated(self):
        root = ET.parse(PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro").getroot()
        xacro_namespace = "{http://www.ros.org/wiki/xacro}"
        argument = root.find(f"./{xacro_namespace}arg[@name='camera_mount']")
        self.assertIsNotNone(argument)
        self.assertEqual(argument.attrib["default"], "fixed")
        camera_layout_defaults = {
            name: root.find(f"./{xacro_namespace}arg[@name='{name}']").attrib[
                "default"
            ]
            for name in (
                "base_camera_mast_xyz",
                "base_camera_xyz",
                "base_camera_rpy",
                "base_camera_image_width",
                "base_camera_image_height",
            )
        }
        self.assertEqual(
            camera_layout_defaults,
            {
                "base_camera_mast_xyz": "-0.35 0.45 0.05",
                "base_camera_xyz": "0 0 1.00",
                "base_camera_rpy": "0 0.543 -0.480",
                "base_camera_image_width": "320",
                "base_camera_image_height": "240",
            },
        )

        camera_macro = root.find(
            f"./{xacro_namespace}macro[@name='strawberry_rgbd_mount']"
        )
        self.assertIsNotNone(camera_macro)
        camera_link = camera_macro.find("link[@name='${link_name}']")
        self.assertIsNotNone(
            camera_link.find("collision[@name='${sensor_name}_housing_collision']")
        )
        joint = camera_macro.find("joint[@name='${joint_name}']")
        self.assertEqual(joint.find("parent").attrib["link"], "${parent_link}")
        self.assertEqual(joint.find("child").attrib["link"], "${link_name}")
        sensor = camera_macro.find(
            "gazebo[@reference='${link_name}']/sensor[@name='${sensor_name}']"
        )
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib["type"], "rgbd_camera")
        self.assertEqual(sensor.findtext("topic"), "${topic}")
        self.assertEqual(
            sensor.findtext("camera/optical_frame_id"),
            "${optical_frame}",
        )

        mount_calls = root.findall(
            f"./{xacro_namespace}if/{xacro_namespace}strawberry_rgbd_mount"
        )
        topics = [call.attrib["topic"] for call in mount_calls]
        self.assertEqual(topics, ["/camera", "/camera/base", "/camera/wrist"])
        self.assertEqual(
            [call.attrib["optical_frame"] for call in mount_calls],
            [
                "strawberry_camera_optical_frame",
                "strawberry_base_camera_optical_frame",
                "strawberry_wrist_camera_optical_frame",
            ],
        )
        self.assertEqual(
            [
                (
                    call.attrib["image_width"],
                    call.attrib["image_height"],
                    call.attrib["update_rate"],
                )
                for call in mount_calls
            ],
            [
                ("640", "480", "30"),
                (
                    "$(arg base_camera_image_width)",
                    "$(arg base_camera_image_height)",
                    "10",
                ),
                ("640", "480", "30"),
            ],
        )
        dual_condition = root.find(
            f"./{xacro_namespace}if[@value=\"${{camera_mount_mode == 'dual'}}\"]"
        )
        self.assertIsNotNone(
            dual_condition.find("link[@name='strawberry_base_camera_mast']")
        )
        mast_joint = dual_condition.find(
            "joint[@name='panda_base_camera_mast_joint']"
        )
        self.assertEqual(mast_joint.find("parent").attrib["link"], "panda_link0")
        self.assertEqual(
            mast_joint.find("origin").attrib,
            {"xyz": "$(arg base_camera_mast_xyz)", "rpy": "0 0 0"},
        )
        base_camera = next(
            call for call in mount_calls if call.attrib["topic"] == "/camera/base"
        )
        self.assertEqual(base_camera.attrib["xyz"], "$(arg base_camera_xyz)")
        self.assertEqual(base_camera.attrib["rpy"], "$(arg base_camera_rpy)")

    def test_panda_has_gazebo_control_contact_and_attachment_plugins(self):
        root = ET.parse(PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro").getroot()
        hardware_plugin = root.findtext(".//ros2_control/hardware/plugin")
        self.assertEqual(hardware_plugin, "gz_ros2_control/GazeboSimSystem")
        gazebo_control_plugin = root.find(
            ".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
        )
        self.assertIsNotNone(gazebo_control_plugin)
        self.assertIsNotNone(
            gazebo_control_plugin.findtext("position_proportional_gain")
        )
        root_gazebo = root.find("./gazebo[@reference='world']")
        self.assertIsNotNone(root_gazebo)
        self.assertEqual(root_gazebo.findtext("static"), "true")
        sensors = root.findall(".//sensor[@type='contact']")
        self.assertEqual(len(sensors), 2)
        detachable = [
            plugin
            for plugin in root.findall(".//plugin")
            if plugin.attrib.get("filename") == "gz-sim-detachable-joint-system"
        ]
        self.assertEqual(len(detachable), 3)
        self.assertEqual(
            {plugin.findtext("child_model") for plugin in detachable},
            {"strawberry_1", "strawberry_2", "strawberry_3"},
        )

        joints = {
            joint.attrib["name"]: tuple(
                float(value) for value in joint.find("origin").attrib["xyz"].split()
            )
            for joint in root.findall(".//joint")
            if joint.attrib.get("name")
            in {
                "panda_left_contact_pad_joint",
                "panda_right_contact_pad_joint",
            }
        }
        self.assertEqual(joints["panda_left_contact_pad_joint"], (0.0, 0.0, 0.027))
        self.assertEqual(joints["panda_right_contact_pad_joint"], (0.0, 0.0, 0.027))

    def test_arm_controller_waits_for_actual_joint_convergence(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "panda_controllers.yaml").read_text(
                encoding="utf-8"
            )
        )
        parameters = config["panda_arm_controller"]["ros__parameters"]
        constraints = parameters["constraints"]
        self.assertEqual(constraints["goal_time"], 8.0)
        self.assertEqual(constraints["stopped_velocity_tolerance"], 0.02)
        for joint_name in parameters["joints"]:
            self.assertEqual(constraints[joint_name]["goal"], 0.05)
            self.assertEqual(constraints[joint_name]["trajectory"], 0.05)


    def test_controller_manager_runs_200hz_to_shrink_command_skew(self):
        """v22/v23: stale-velocity drift at recovery home crossed tolerance.

        The plugin recomputes its velocity law once per controller cycle;
        the observed recovery-home drift (~0.0035 rad/s accumulation over
        ~2 s) lives in the gap between JTC's reference and the plugin's
        last-applied command. Doubling the controller rate halves that
        skew window per cycle. Single variable: update_rate 100 -> 200.
        """
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "panda_controllers.yaml").read_text(
                encoding="utf-8"
            )
        )
        manager = config["controller_manager"]["ros__parameters"]
        self.assertEqual(manager["update_rate"], 200)

    def test_gripper_stall_window_allows_first_measured_motion(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "panda_controllers.yaml").read_text(
                encoding="utf-8"
            )
        )
        controller_joints = {
            "panda_gripper_controller": "panda_finger_joint1",
            "panda_gripper_right_controller": "panda_finger_joint2",
        }
        for controller_name, joint_name in controller_joints.items():
            parameters = config[controller_name]["ros__parameters"]
            self.assertEqual(parameters["joint"], joint_name)
            self.assertTrue(parameters["allow_stalling"])
            self.assertEqual(parameters["stall_timeout"], 1.0)
            self.assertEqual(parameters["goal_tolerance"], 0.003)
            self.assertIn(
                controller_name,
                config["controller_manager"]["ros__parameters"],
            )
        launch_text = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"panda_gripper_controller"', launch_text)
        self.assertIn('"panda_gripper_right_controller"', launch_text)

    def test_both_finger_joints_have_explicit_position_command_interfaces(self):
        root = ET.parse(PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro").getroot()
        ros2_control = root.find(".//ros2_control")
        self.assertIsNotNone(ros2_control)
        finger_joints = {
            joint.attrib["name"]: joint
            for joint in ros2_control.findall("joint")
            if joint.attrib.get("name", "").startswith("panda_finger_joint")
        }
        self.assertEqual(
            set(finger_joints),
            {"panda_finger_joint1", "panda_finger_joint2"},
        )
        for joint in finger_joints.values():
            self.assertIsNotNone(
                joint.find("./command_interface[@name='position']")
            )
        self.assertEqual(
            finger_joints["panda_finger_joint2"].attrib.get("mimic"),
            "false",
        )

    def test_wrist_depth_bridge_has_bounded_burst_queue(self):
        bridges = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "bridge.yaml").read_text(
                encoding="utf-8"
            )
        )
        wrist_depth = next(
            item
            for item in bridges
            if item.get("ros_topic_name")
            == "/camera/wrist/depth/image_raw"
        )
        self.assertEqual(wrist_depth["direction"], "GZ_TO_ROS")
        self.assertEqual(wrist_depth["qos_profile"], "SENSOR_DATA")
        self.assertEqual(wrist_depth["publisher_queue"], 30)

    def test_world_fruit_poses_match_scene_manifest(self):
        root = ET.parse(PACKAGE_ROOT / "worlds" / "strawberry_orchard.sdf").getroot()
        world_poses = {
            include.findtext("name"): tuple(
                float(value) for value in include.findtext("pose").split()[:3]
            )
            for include in root.findall("./world/include")
            if include.findtext("name", "").startswith("strawberry_")
            and include.findtext("name", "")[-1:].isdigit()
        }
        manifest = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "scene.yaml").read_text(encoding="utf-8")
        )
        manifest_poses = {
            fruit["model_name"]: tuple(float(value) for value in fruit["initial_pose_m"])
            for fruit in manifest["fruits"]
        }
        self.assertEqual(world_poses, manifest_poses)

    def test_scene_manifests_split_v1_and_v2_fruit_geometry(self):
        canonical = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "scene.yaml").read_text(
                encoding="utf-8"
            )
        )
        archived = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "scene_tabletop_v1.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(canonical["fruit_collision_radius_m"], 0.026)
        self.assertNotIn("fruit_collision_radius_m", archived)
        self.assertEqual(
            load_scene_config(
                PACKAGE_ROOT / "config" / "scene_tabletop_v1.yaml"
            ).fruit_collision_radius_m,
            0.035,
        )

    def test_fruits_disable_gravity_but_decay_collision_impulses(self):
        for model_name in ("strawberry_ripe", "strawberry_unripe"):
            root = ET.parse(
                PACKAGE_ROOT / "models" / model_name / "model.sdf"
            ).getroot()
            link = root.find("./model/link[@name='fruit_link']")
            self.assertIsNotNone(link)
            self.assertEqual(link.findtext("gravity"), "false")
            self.assertGreater(float(link.findtext("velocity_decay/linear")), 0.0)
            self.assertGreater(float(link.findtext("velocity_decay/angular")), 0.0)
            sensor = link.find("sensor[@name='fruit_contact_sensor']")
            self.assertIsNotNone(sensor)
            self.assertEqual(sensor.attrib["type"], "contact")
            self.assertEqual(
                sensor.findtext("contact/collision"), "fruit_collision"
            )
            self.assertEqual(
                sensor.findtext("contact/topic"),
                "/strawberry/sim/fruit_contacts",
            )

    def test_contact_frames_are_collision_free_and_sensors_use_stock_fingers(self):
        root = ET.parse(PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro").getroot()
        for side in ("left", "right"):
            joint = root.find(
                f"./joint[@name='panda_{side}_contact_pad_joint']/origin"
            )
            self.assertIsNotNone(joint)
            self.assertEqual(joint.attrib["xyz"], "0 0 0.027")
            pad_link = root.find(
                f"./link[@name='panda_{side}_contact_pad']"
            )
            self.assertIsNotNone(pad_link)
            self.assertEqual(pad_link.findall("collision"), [])
            sensor = root.find(
                f"./gazebo[@reference='panda_{side}finger']"
                f"/sensor[@name='{side}_contact_sensor']"
            )
            self.assertIsNotNone(sensor)
            self.assertEqual(
                sensor.findtext("contact/collision"),
                f"panda_{side}finger_collision",
            )

    @unittest.skipUnless(
        shutil.which("xacro") and shutil.which("gz"),
        "xacro and Gazebo are required for the conversion-level check",
    )
    def test_finger_contact_sensor_references_survive_urdf_to_sdf_conversion(self):
        try:
            from ament_index_python.packages import get_package_share_directory

            get_package_share_directory("strawberry_sim")
        except (ImportError, LookupError):
            self.skipTest("strawberry_sim is not present in the ament index")
        xacro_path = PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro"
        initial_positions = PACKAGE_ROOT / "config" / "panda_initial_positions.yaml"
        expanded = subprocess.run(
            [
                "xacro",
                str(xacro_path),
                f"initial_positions_file:={initial_positions}",
                "enable_attachment:=false",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", encoding="utf-8"
        ) as temporary_urdf:
            temporary_urdf.write(expanded)
            temporary_urdf.flush()
            converted = subprocess.run(
                ["gz", "sdf", "-p", temporary_urdf.name],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        root = ET.fromstring(converted)
        world_joint = root.find("./model/joint[@name='panda_world_joint']")
        self.assertIsNotNone(world_joint)
        self.assertEqual(world_joint.findtext("parent"), "world")
        self.assertEqual(world_joint.findtext("child"), "panda_link0")
        for side in ("left", "right"):
            link = root.find(
                f"./model/link[@name='panda_{side}finger']"
            )
            self.assertIsNotNone(link)
            collision_names = {
                collision.attrib["name"] for collision in link.findall("collision")
            }
            sensor_reference = link.findtext("sensor/contact/collision")
            self.assertIn(sensor_reference, collision_names)
            sensor_collision = next(
                collision
                for collision in link.findall("collision")
                if collision.attrib["name"] == sensor_reference
            )
            collision_pose = tuple(
                float(value)
                for value in sensor_collision.findtext(
                    "pose", "0 0 0 0 0 0"
                ).split()
            )
            self.assertEqual(collision_pose[:5], (0.0, 0.0, 0.0, 0.0, 0.0))
            if side == "left":
                self.assertAlmostEqual(collision_pose[5], 0.0)
            else:
                self.assertAlmostEqual(abs(collision_pose[5]), 3.141592653589793)
            self.assertEqual(
                link.findtext("sensor/contact/topic"),
                f"/strawberry/sim/gripper/{side}_contacts",
            )

    @unittest.skipUnless(
        shutil.which("xacro"),
        "xacro is required for the robot-camera expansion check",
    )
    def test_robot_camera_modes_expand_to_distinct_links_frames_and_topics(self):
        try:
            from ament_index_python.packages import get_package_share_directory

            get_package_share_directory("strawberry_sim")
        except (ImportError, LookupError):
            self.skipTest("strawberry_sim is not present in the ament index")
        xacro_path = PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro"
        initial_positions = PACKAGE_ROOT / "config" / "panda_initial_positions.yaml"

        expected = {
            "fixed": [],
            "wrist": [
                (
                    "strawberry_camera_link",
                    "strawberry_camera_optical_frame",
                    "/camera",
                )
            ],
            "dual": [
                (
                    "strawberry_base_camera_link",
                    "strawberry_base_camera_optical_frame",
                    "/camera/base",
                ),
                (
                    "strawberry_wrist_camera_link",
                    "strawberry_wrist_camera_optical_frame",
                    "/camera/wrist",
                ),
            ],
        }
        for mode, cameras in expected.items():
            with self.subTest(camera_mount=mode):
                expanded = subprocess.run(
                    [
                        "xacro",
                        str(xacro_path),
                        f"initial_positions_file:={initial_positions}",
                        "enable_attachment:=false",
                        f"camera_mount:={mode}",
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                ).stdout
                robot = ET.fromstring(expanded)
                for link_name, optical_frame, topic in cameras:
                    link = robot.find(f"./link[@name='{link_name}']")
                    self.assertIsNotNone(link)
                    self.assertIsNotNone(link.find("collision"))
                    self.assertIsNotNone(
                        robot.find(f"./link[@name='{optical_frame}']")
                    )
                    sensor = robot.find(
                        f"./gazebo[@reference='{link_name}']/sensor"
                    )
                    self.assertEqual(sensor.findtext("topic"), topic)
                    self.assertEqual(
                        sensor.findtext("camera/optical_frame_id"), optical_frame
                    )
                camera_topics = {
                    element.text
                    for element in robot.findall("./gazebo/sensor/topic")
                    if element.text and element.text.startswith("/camera")
                }
                self.assertEqual(camera_topics, {camera[2] for camera in cameras})
                if mode == "dual":
                    self.assertIsNotNone(
                        robot.find("./link[@name='strawberry_base_camera_mast']")
                    )

    @unittest.skipUnless(
        shutil.which("xacro"),
        "xacro is required for the camera-resolution expansion check",
    )
    def test_dual_base_camera_resolution_profile_expands_into_sensor(self):
        try:
            from ament_index_python.packages import get_package_share_directory

            get_package_share_directory("strawberry_sim")
        except (ImportError, LookupError):
            self.skipTest("strawberry_sim is not present in the ament index")
        xacro_path = PACKAGE_ROOT / "urdf" / "panda_gz.urdf.xacro"
        initial_positions = PACKAGE_ROOT / "config" / "panda_initial_positions.yaml"
        expanded = subprocess.run(
            [
                "xacro",
                str(xacro_path),
                f"initial_positions_file:={initial_positions}",
                "enable_attachment:=false",
                "camera_mount:=dual",
                "base_camera_image_width:=640",
                "base_camera_image_height:=480",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        robot = ET.fromstring(expanded)
        sensor = robot.find(
            "./gazebo[@reference='strawberry_base_camera_link']/sensor"
        )
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "480")


if __name__ == "__main__":
    unittest.main()
