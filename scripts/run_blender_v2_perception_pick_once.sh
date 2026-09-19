#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/development/blender_v2_perception_pick_once_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-227}"
expected_model_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
target_topic="/strawberry/perception/target_pose"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ -f "${model_path}" ]] || {
  echo "Model is missing: ${model_path}" >&2
  exit 2
}
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_model_sha}" ]] || {
  echo "Wrong model hash" >&2
  exit 2
}
[[ ! -e "${output_dir}" ]] || {
  echo "Output exists; choose a fresh directory: ${output_dir}" >&2
  exit 4
}
[[ "${domain_id}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]] || {
  echo "ROS domain ID must be in [0,232]" >&2
  exit 2
}

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"
mkdir -p "${output_dir}"

launch_pid=""
pipeline_pids=()

stop_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 -- "-${pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 300); do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

shutdown_all() {
  local pid
  for pid in "${pipeline_pids[@]}"; do
    stop_group "${pid}"
  done
  pipeline_pids=()
  stop_group "${launch_pid}"
  launch_pid=""
}
trap shutdown_all EXIT INT TERM

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  camera_mount:=dual \
  enable_attachment:=true \
  enable_pose_control:=false \
  start_perception:=false \
  start_oracle_provider:=false \
  start_manipulation:=true \
  start_orchestrator:=false \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

timeout --signal=TERM 120 python \
  "${repo_root}/scripts/move_wrist_observation_pose.py" \
  --camera-mount dual \
  --planning-attempts 3 \
  --qx 0.0 --qy 0.9537169507 --qz 0.0 --qw 0.3007057995 \
  --output "${output_dir}/observation_motion.json" \
  >"${output_dir}/observation_motion.log" 2>&1

perception_config="$(
  ros2 pkg prefix strawberry_perception
)/share/strawberry_perception/config/perception.yaml"
localization_config="$(
  ros2 pkg prefix strawberry_localization
)/share/strawberry_localization/config/localization_blender_v2.yaml"

# Start the wrist pipeline only after the observation move. This prevents
# pre-motion TargetPose samples from entering the live handoff queue.
setsid ros2 run strawberry_perception perception_node --ros-args \
  --params-file "${perception_config}" \
  -r __node:=strawberry_pick_wrist_perception \
  -p use_sim_time:=true \
  -p model_path:="${model_path}" \
  -p confidence_threshold:=0.58 \
  -p image_topic:=/camera/wrist/color/image_raw \
  -p detections_topic:=/strawberry/perception/detections \
  >"${output_dir}/perception.log" 2>&1 &
pipeline_pids+=("$!")

setsid ros2 run strawberry_localization localization_node --ros-args \
  --params-file "${localization_config}" \
  -r __node:=strawberry_pick_wrist_localization \
  -p use_sim_time:=true \
  -p confidence_threshold:=0.58 \
  -p detections_topic:=/strawberry/perception/detections \
  -p target_pose_topic:="${target_topic}" \
  -p depth_topic:=/camera/wrist/depth/image_raw \
  -p camera_info_topic:=/camera/wrist/camera_info \
  -p surface_to_center_offset_m:=0.026 \
  -p sensor_qos_depth:=30 \
  -p selection_roi_min_x_px:=320 \
  -p selection_roi_min_y_px:=240 \
  -p selection_roi_max_x_px:=640 \
  -p selection_roi_max_y_px:=480 \
  -p allow_stationary_latest_tf_fallback:=true \
  -p allow_stationary_sensor_sync_fallback:=true \
  -p stationary_sync_tolerance_sec:=0.105 \
  >"${output_dir}/localization.log" 2>&1 &
pipeline_pids+=("$!")

timeout --signal=TERM 180 ros2 run strawberry_manipulation \
  handoff_shadow_probe \
  --output-json "${output_dir}/handoff_shadow.json" \
  --expected-target-id 1 \
  --target-topic "${target_topic}" \
  --camera-mount dual \
  --timeout-sec 90 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/handoff_shadow.log" 2>&1

timeout --signal=TERM 180 ros2 run strawberry_manipulation \
  pregrasp_planning_shadow \
  --handoff-json "${output_dir}/handoff_shadow.json" \
  --output-json "${output_dir}/pregrasp_shadow.json" \
  --target-topic "${target_topic}" \
  --camera-mount dual \
  --timeout-sec 90 \
  --planning-attempts 3 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/pregrasp_shadow.log" 2>&1

python "${repo_root}/scripts/evaluate_blender_v2_perception_execution_readiness.py" \
  --handoff "${output_dir}/handoff_shadow.json" \
  --no-motion-gate "${output_dir}/pregrasp_shadow.json" \
  --scene "${repo_root}/ros2_ws/src/strawberry_sim/config/scene.yaml" \
  --geometry-gate \
    "${repo_root}/results/development/blender_v2_gripper_geometry_sweep_v1/summary.json" \
  --output "${output_dir}/execution_readiness.json" \
  >"${output_dir}/execution_readiness.log" 2>&1

# Exactly one perception-derived simulation action is attempted after every
# same-world no-motion and geometry check above has passed.
set +e
timeout --signal=TERM 300 python \
  "${repo_root}/scripts/test_oracle_pick_and_place.py" \
  --target-id 1 \
  --target-topic "${target_topic}" \
  --target-message-type target_pose \
  --place-x 0.35 --place-y -0.45 --place-z 0.45 \
  --startup-timeout-sec 90 \
  --action-timeout-sec 180 \
  --stability-samples 10 \
  --stability-tolerance-m 0.001 \
  --output "${output_dir}/trial_01.json" \
  >"${output_dir}/client.log" 2>&1
client_status=$?
set -e

shutdown_all
trap - EXIT INT TERM

if [[ "${client_status}" -ne 0 ]]; then
  echo "Perception pick failed with client status ${client_status}" >&2
  exit "${client_status}"
fi
echo "Perception pick result: ${output_dir}/trial_01.json"
