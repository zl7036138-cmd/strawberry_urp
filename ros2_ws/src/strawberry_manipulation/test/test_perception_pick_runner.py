from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
RUNNER = (
    REPOSITORY_ROOT / "scripts" / "run_blender_v2_perception_pick_once.sh"
)


def test_runner_gates_one_perception_pick_without_oracle_provider():
    source = RUNNER.read_text(encoding="utf-8")
    readiness = source.index(
        "evaluate_blender_v2_perception_execution_readiness.py"
    )
    action = source.index("test_oracle_pick_and_place.py")
    assert readiness < action
    assert source.count("test_oracle_pick_and_place.py") == 1
    assert "start_oracle_provider:=false" in source
    assert "start_perception:=false" in source
    assert source.index("move_wrist_observation_pose.py") < source.index(
        "strawberry_pick_wrist_perception"
    )
    assert 'target_topic="/strawberry/perception/target_pose"' in source
    assert "--target-topic \"${target_topic}\"" in source
    assert "--target-message-type target_pose" in source
    assert "enable_attachment:=true" in source
    assert "surface_to_center_offset_m:=0.026" in source
