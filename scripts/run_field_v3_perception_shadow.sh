#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/field_v3_perception_shadow_v6}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-226}"
expected_model_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ -f "${model_path}" ]] || {
  echo "Model is missing: ${model_path}" >&2
  exit 2
}
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_model_sha}" ]] || {
  echo "Model hash does not match the accepted runtime checkpoint" >&2
  exit 2
}
[[ ! -e "${output_dir}" ]] || {
  echo "Output already exists; choose a fresh directory: ${output_dir}" >&2
  exit 4
}

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"
mkdir -p "${output_dir}"

simulation_share="$(ros2 pkg prefix strawberry_sim)/share/strawberry_sim"
world_file="${simulation_share}/worlds/strawberry_field_v3.sdf"
scene_config="${simulation_share}/config/scene_field_v3.yaml"
initial_positions="${simulation_share}/config/panda_initial_positions_field_v3.yaml"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 200); do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  world_file:="${world_file}" \
  scene_config_file:="${scene_config}" \
  initial_positions_file:="${initial_positions}" \
  simulation_seed:=20260728 \
  camera_mount:=dual \
  start_perception:=true \
  start_oracle_provider:=false \
  start_manipulation:=false \
  start_orchestrator:=false \
  enable_attachment:=false \
  enable_pose_control:=false \
  model_path:="${model_path}" \
  confidence_threshold:=0.58 \
  allow_stationary_latest_tf_fallback:=true \
  perception_image_topic:=/camera/wrist/color/image_raw \
  localization_depth_topic:=/camera/wrist/depth/image_raw \
  localization_camera_info_topic:=/camera/wrist/camera_info \
  localization_selection_roi_min_x_px:=320 \
  localization_selection_roi_min_y_px:=240 \
  localization_selection_roi_max_x_px:=640 \
  localization_selection_roi_max_y_px:=480 \
  perception_detections_topic:=/strawberry/shadow/detections \
  perception_target_topic:=/strawberry/shadow/target_pose \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/shadow_window.json" \
  --scenario-id field_v3_wrist_target_1_roi_v1 \
  --lighting nominal --occlusion natural_field \
  --warmup-frames 10 \
  --ready-consecutive-target-pose-frames 15 \
  --frames 60 --timeout-sec 140 \
  --post-window-wait-sec 2 \
  --window-boundary after_field_v3_spawn_before_any_robot_motion \
  >"${output_dir}/shadow_window.log" 2>&1

timeout --signal=TERM 180 python \
  "${repo_root}/scripts/capture_target_pose_accuracy.py" \
  --output "${output_dir}/target_pose_accuracy.json" \
  --expected-target-id 1 \
  --samples 60 --timeout-sec 120 \
  >"${output_dir}/target_pose_accuracy.log" 2>&1

timeout --signal=TERM 180 ros2 run strawberry_manipulation handoff_shadow_probe \
  --output-json "${output_dir}/collision_scene_audit.json" \
  --expected-target-id 1 \
  --scene-config "${scene_config}" \
  --target-topic /strawberry/shadow/target_pose \
  --camera-mount dual \
  --timeout-sec 60 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/collision_scene_audit.log" 2>&1

timeout --signal=TERM 180 ros2 run strawberry_manipulation pregrasp_planning_shadow \
  --handoff-json "${output_dir}/collision_scene_audit.json" \
  --output-json "${output_dir}/pregrasp_planning_shadow.json" \
  --scene-config "${scene_config}" \
  --target-topic /strawberry/shadow/target_pose \
  --camera-mount dual \
  --timeout-sec 60 \
  --planning-attempts 3 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/pregrasp_planning_shadow.log" 2>&1

python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/wrist/color/image_raw \
  --output "${output_dir}/wrist_rgb.png" --timeout-sec 30 \
  >"${output_dir}/wrist_capture.log" 2>&1

shutdown_launch
trap - EXIT INT TERM
if grep -Eqi \
  "Traceback|process has died|RuntimeError|exception was never retrieved" \
  "${output_dir}/launch.log" \
  "${output_dir}/shadow_window.log" \
  "${output_dir}/target_pose_accuracy.log" \
  "${output_dir}/collision_scene_audit.log" \
  "${output_dir}/pregrasp_planning_shadow.log"; then
  echo "Field-v3 perception Shadow logs contain an unhandled failure" >&2
  exit 1
fi
echo "Field-v3 perception Shadow: ${output_dir}"
