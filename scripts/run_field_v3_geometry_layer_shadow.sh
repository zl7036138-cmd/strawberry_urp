#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/field_v3_geometry_layer_shadow_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-231}"
expected_model_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
detections_topic="/strawberry/geometry_shadow/detections"
target_topic="/strawberry/geometry_shadow/target_pose"

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

simulation_share="$(ros2 pkg prefix strawberry_sim)/share/strawberry_sim"
perception_share="$(ros2 pkg prefix strawberry_perception)/share/strawberry_perception"
localization_share="$(ros2 pkg prefix strawberry_localization)/share/strawberry_localization"
world_file="${simulation_share}/worlds/strawberry_field_v3.sdf"
scene_config="${simulation_share}/config/scene_field_v3.yaml"
perception_config="${perception_share}/config/perception.yaml"
localization_config="${localization_share}/config/localization_geometry_layer_v1.yaml"
initial_positions_source="${repo_root}/ros2_ws/src/strawberry_sim/config/panda_initial_positions_field_v3.yaml"
runtime_config_dir="${HOME}/.cache/strawberry_urp/runtime"
mkdir -p "${runtime_config_dir}"
initial_positions="${runtime_config_dir}/panda_initial_positions_field_v3.yaml"
cp "${initial_positions_source}" "${initial_positions}"

launch_pid=""
pipeline_pids=()
stop_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 -- "-${pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 200); do
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
  world_file:="${world_file}" \
  scene_config_file:="${scene_config}" \
  initial_positions_file:="${initial_positions}" \
  simulation_seed:=20260728 \
  camera_mount:=dual \
  start_perception:=false \
  start_oracle_provider:=false \
  start_manipulation:=false \
  start_orchestrator:=false \
  enable_attachment:=false \
  enable_pose_control:=false \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

setsid ros2 run strawberry_perception perception_node --ros-args \
  --params-file "${perception_config}" \
  -p use_sim_time:=true \
  -p model_path:="${model_path}" \
  -p confidence_threshold:=0.31 \
  -p image_topic:=/camera/wrist/color/image_raw \
  -p detections_topic:="${detections_topic}" \
  >"${output_dir}/perception.log" 2>&1 &
pipeline_pids+=("$!")

setsid ros2 run strawberry_localization localization_node --ros-args \
  --params-file "${localization_config}" \
  -p use_sim_time:=true \
  -p confidence_threshold:=0.31 \
  -p detections_topic:="${detections_topic}" \
  -p target_pose_topic:="${target_topic}" \
  -p depth_topic:=/camera/wrist/depth/image_raw \
  -p camera_info_topic:=/camera/wrist/camera_info \
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

initial_pose_ready="false"
for _ in $(seq 1 180); do
  if grep -Fq "found initial value: 1.310378" "${output_dir}/launch.log" &&
    grep -Fq "found initial value: -1.599778" "${output_dir}/launch.log"; then
    initial_pose_ready="true"
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Simulation exited before the field observation pose was ready" >&2
    exit 5
  fi
  sleep 0.5
done
[[ "${initial_pose_ready}" == "true" ]] || {
  echo "Field observation pose was not applied" >&2
  exit 5
}

timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/shadow_window.json" \
  --scenario-id field_v3_geometry_layer_target_1_v1 \
  --lighting nominal --occlusion natural_field \
  --warmup-frames 10 \
  --ready-consecutive-target-pose-frames 15 \
  --frames 60 --timeout-sec 140 \
  --post-window-wait-sec 2 \
  --window-boundary geometry_layer_before_any_robot_motion \
  --ros-args \
  -r /strawberry/shadow/detections:="${detections_topic}" \
  -r /strawberry/shadow/target_pose:="${target_topic}" \
  >"${output_dir}/shadow_window.log" 2>&1

timeout --signal=TERM 180 python \
  "${repo_root}/scripts/capture_target_pose_accuracy.py" \
  --output "${output_dir}/target_pose_accuracy.json" \
  --target-topic "${target_topic}" \
  --expected-target-id 1 \
  --samples 60 --timeout-sec 120 \
  >"${output_dir}/target_pose_accuracy.log" 2>&1

timeout --signal=TERM 180 ros2 run strawberry_manipulation handoff_shadow_probe \
  --output-json "${output_dir}/collision_scene_audit.json" \
  --expected-target-id 1 \
  --scene-config "${scene_config}" \
  --target-topic "${target_topic}" \
  --camera-mount dual \
  --timeout-sec 60 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/collision_scene_audit.log" 2>&1

python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/wrist/color/image_raw \
  --output "${output_dir}/wrist_rgb.png" --timeout-sec 30 \
  >"${output_dir}/wrist_capture.log" 2>&1

python "${repo_root}/scripts/summarize_field_v3_geometry_layer_shadow.py" \
  --run-directory "${output_dir}" \
  >"${output_dir}/summary.log" 2>&1

shutdown_all
trap - EXIT INT TERM
if grep -Eqi \
  "Traceback|process has died|RuntimeError|exception was never retrieved" \
  "${output_dir}/launch.log" \
  "${output_dir}/perception.log" \
  "${output_dir}/localization.log" \
  "${output_dir}/shadow_window.log" \
  "${output_dir}/target_pose_accuracy.log" \
  "${output_dir}/collision_scene_audit.log"; then
  echo "Geometry-layer Shadow logs contain an unhandled failure" >&2
  exit 1
fi
echo "Field-v3 geometry-layer Shadow: ${output_dir}"
