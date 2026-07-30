import pathlib
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config"
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def test_srdf_has_fixed_base_arm_group_and_no_virtual_joint():
    root = ET.parse(CONFIG / "panda.srdf").getroot()
    arm = next(group for group in root.findall("group") if group.attrib["name"] == "panda_arm")
    chain = arm.find("chain")
    assert chain.attrib == {"base_link": "panda_link0", "tip_link": "panda_link8"}
    assert root.find("virtual_joint") is None
    states = {
        state.attrib["name"] for state in root.findall("group_state")
        if state.attrib["group"] == "panda_arm"
    }
    assert states == {"ready", "smoke"}
    disabled_pairs = {
        frozenset((entry.attrib["link1"], entry.attrib["link2"]))
        for entry in root.findall("disable_collisions")
    }
    assert frozenset(("panda_link5", "panda_link7")) in disabled_pairs
    assert frozenset(("panda_link5", "panda_hand")) in disabled_pairs


def test_moveit_controller_matches_gazebo_arm_controller():
    config = yaml.safe_load((CONFIG / "moveit_controllers.yaml").read_text())
    manager = config["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["panda_arm_controller"]
    assert manager["panda_arm_controller"]["action_ns"] == "follow_joint_trajectory"
    assert len(manager["panda_arm_controller"]["joints"]) == 7


def test_moveit_py_uses_sim_time_and_bounded_planning():
    config = yaml.safe_load((CONFIG / "moveit_py.yaml").read_text())
    assert config["use_sim_time"] is True
    clock_qos = config["qos_overrides"]["/clock"]["subscription"]
    assert clock_qos["durability"] == "volatile"
    assert clock_qos["reliability"] == "best_effort"
    request = config["plan_request_params"]
    assert request["planning_time"] <= 5.0
    assert request["planner_id"] == "RRTConnectkConfigDefault"


def test_wrist_observation_motion_loads_every_scene_fruit_as_an_obstacle():
    source = (
        REPOSITORY_ROOT / "scripts" / "move_wrist_observation_pose.py"
    ).read_text(encoding="utf-8")
    assert "from strawberry_sim.core import load_scene_config" in source
    assert "for fruit in scene.ordered_fruits" in source
    assert "fruit_obstacles=fruit_obstacles" in source
    assert "fruit_obstacles={}" not in source


def test_wrist_shadow_runner_exposes_two_bounded_observation_presets():
    source = (
        REPOSITORY_ROOT / "scripts" / "run_wrist_observation_shadow.sh"
    ).read_text(encoding="utf-8")
    assert 'observation_pose="${5:-center}"' in source
    assert "blender_v2_wrist_observation_center" in source
    assert "blender_v2_wrist_observation_lower" in source
    assert "--qy 0.9537169507" in source


def test_dual_sequence_stops_base_pipeline_before_wrist_motion():
    source = (
        REPOSITORY_ROOT / "scripts" / "run_dual_sequential_observation.sh"
    ).read_text(encoding="utf-8")
    stop_index = source.index("stop_pipeline\n\npose_args=()")
    motion_index = source.index("move_wrist_observation_pose.py")
    wrist_index = source.index("start_wrist_pipeline \\\n")
    assert stop_index < motion_index < wrist_index
    assert "dual_observation_selector" in source
    assert "dual_observation_summary" in source
    assert source.count("-p use_sim_time:=true") == 6
    assert "handoff_shadow_probe" in source
    assert "--handoff-shadow-json" in source
    assert "pregrasp_planning_shadow" in source
    assert "--pregrasp-shadow-json" in source
    assert "--planning-attempts 3" in source
    assert "localization_blender_v2.yaml" in source


def test_runtime_collision_users_take_radius_from_scene_manifest():
    for relative_path in (
        "strawberry_manipulation/action_server.py",
        "strawberry_manipulation/handoff_shadow.py",
        "strawberry_manipulation/pregrasp_shadow.py",
    ):
        source = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
        assert "scene.fruit_collision_radius_m" in source


def test_runtime_collision_users_take_static_profile_from_scene_manifest():
    for relative_path in (
        "strawberry_manipulation/action_server.py",
        "strawberry_manipulation/handoff_shadow.py",
        "strawberry_manipulation/pregrasp_shadow.py",
    ):
        source = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
        assert "static_collision_objects" in source
        assert "scene.static_collision_profile" in source

    observation_source = (
        REPOSITORY_ROOT / "scripts" / "move_wrist_observation_pose.py"
    ).read_text(encoding="utf-8")
    assert "static_collision_objects" in observation_source
    assert "scene.static_collision_profile" in observation_source


def test_execution_and_planning_shadow_share_scene_bound_grasp_geometry():
    action_source = (
        PACKAGE_ROOT / "strawberry_manipulation" / "action_server.py"
    ).read_text(encoding="utf-8")
    shadow_source = (
        PACKAGE_ROOT / "strawberry_manipulation" / "pregrasp_shadow.py"
    ).read_text(encoding="utf-8")
    for source in (action_source, shadow_source):
        assert "load_grasp_geometry" in source
        assert "scene.world_name" in source
        assert "scene.fruit_collision_radius_m" in source
        assert "grasp_geometry.tool_center_offset_m" in source
    assert "grasp_geometry.gripper_closed_width_m_per_finger" in action_source
    assert '"tool_center_offset_m": 0.1054' not in shadow_source
