#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/development/p4_qualification_runtime_smoke_v1}"
model_path="${repo_root}/outputs/perception/yolo11s_640_sim_adapt_v1/weights/best.pt"
world_path="${output_dir}/world.sdf"
receipt_path="${output_dir}/condition_receipt.json"

if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Smoke output is not empty: ${output_dir}" >&2
  exit 4
fi
mkdir -p "${output_dir}"
python3 "${repo_root}/scripts/materialize_scene_condition.py" \
  --base-world "${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf" \
  --config "${repo_root}/config/t70_scene_conditions_v2.json" \
  --lighting nominal --occlusion none --seed 20260710 \
  --target-position-m 0.455 -0.065 0.50 \
  --output-world "${world_path}" --output-receipt "${receipt_path}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID=199

launch_pid=""
shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then return; fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then kill -KILL -- "-${launch_pid}" 2>/dev/null || true; fi
  wait "${launch_pid}" 2>/dev/null || true
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true world_file:="${world_path}" simulation_seed:=20260718 \
  enable_pose_control:=true start_perception:=true start_oracle_provider:=false \
  start_manipulation:=false start_orchestrator:=false enable_attachment:=false \
  model_path:="${model_path}" confidence_threshold:=0.80 \
  perception_detections_topic:=/strawberry/shadow/detections \
  perception_target_topic:=/strawberry/shadow/target_pose \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
  --receipt "${receipt_path}" --output-json "${output_dir}/condition_probe.json" \
  --sensor-timeout-sec 45 >"${output_dir}/condition_probe.log" 2>&1
timeout --signal=TERM 90 python "${repo_root}/scripts/configure_p4_qualification_scene.py" \
  --target-id 1 --maturity RIPE \
  --model-pose strawberry_1 1 0.455 -0.065 0.50 \
  --model-pose strawberry_2 2 0.20 -2.00 1.00 \
  --model-pose strawberry_3 3 0.40 -2.00 1.00 \
  --settled-camera-frames 10 --output "${output_dir}/scene_configuration.json" \
  >"${output_dir}/scene_configuration.log" 2>&1
timeout --signal=TERM 90 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/shadow_window.json" \
  --scenario-id p4_runtime_smoke_nonmatrix --lighting nominal --occlusion none \
  --frames 5 --timeout-sec 45 --post-window-wait-sec 0.2 \
  --window-boundary development_smoke_before_robot_motion \
  >"${output_dir}/shadow_window.log" 2>&1
shutdown_launch
trap - EXIT INT TERM
touch "${output_dir}/shutdown.ok"
python3 -c 'import json,sys; p=json.load(open(sys.argv[1])); s=json.load(open(sys.argv[2])); assert p["robot_motion_started"] is False; assert s["summary"]["frame_count"] == 5; print(json.dumps({"runtime_smoke_passed": True, "ripe_frames": s["summary"]["frames_with_ripe_detection"], "target_pose_frames": s["summary"]["frames_with_target_pose"]}, sort_keys=True))' \
  "${output_dir}/scene_configuration.json" "${output_dir}/shadow_window.json"
