#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/development/dual_sequential_observation_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-222}"
expected_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ -f "${model_path}" ]] || {
  echo "Model missing: ${model_path}" >&2
  exit 2
}
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_sha}" ]] || {
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

perception_config="$(
  ros2 pkg prefix strawberry_perception
)/share/strawberry_perception/config/perception.yaml"
localization_config="$(
  ros2 pkg prefix strawberry_localization
)/share/strawberry_localization/config/localization_blender_v2.yaml"

sim_pid=""
pipeline_pids=()

stop_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 -- "-${pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

stop_pipeline() {
  local pid
  for pid in "${pipeline_pids[@]}"; do
    stop_group "${pid}"
  done
  pipeline_pids=()
}

shutdown_all() {
  stop_pipeline
  stop_group "${sim_pid}"
  sim_pid=""
}
trap shutdown_all EXIT INT TERM

start_base_pipeline() {
  setsid ros2 run strawberry_perception perception_node --ros-args \
    --params-file "${perception_config}" \
    -r __node:=strawberry_base_overview_perception \
    -p use_sim_time:=true \
    -p model_path:="${model_path}" \
    -p confidence_threshold:=0.58 \
    -p image_topic:=/camera/base/color/image_raw \
    -p detections_topic:=/strawberry/shadow/detections \
    >"${output_dir}/base_perception.log" 2>&1 &
  pipeline_pids+=("$!")

  setsid ros2 run strawberry_localization localization_node --ros-args \
    --params-file "${localization_config}" \
    -r __node:=strawberry_base_overview_localization \
    -p use_sim_time:=true \
    -p confidence_threshold:=0.58 \
    -p detections_topic:=/strawberry/shadow/detections \
    -p target_pose_topic:=/strawberry/shadow/target_pose \
    -p depth_topic:=/camera/base/depth/image_raw \
    -p camera_info_topic:=/camera/base/camera_info \
    -p allow_stationary_latest_tf_fallback:=false \
    >"${output_dir}/base_localization.log" 2>&1 &
  pipeline_pids+=("$!")
}

start_wrist_pipeline() {
  local roi_min_x="$1"
  local roi_min_y="$2"
  local roi_max_x="$3"
  local roi_max_y="$4"

  setsid ros2 run strawberry_perception perception_node --ros-args \
    --params-file "${perception_config}" \
    -r __node:=strawberry_wrist_attention_perception \
    -p use_sim_time:=true \
    -p model_path:="${model_path}" \
    -p confidence_threshold:=0.58 \
    -p image_topic:=/camera/wrist/color/image_raw \
    -p detections_topic:=/strawberry/shadow/detections \
    >"${output_dir}/wrist_perception.log" 2>&1 &
  pipeline_pids+=("$!")

  setsid ros2 run strawberry_localization localization_node --ros-args \
    --params-file "${localization_config}" \
    -r __node:=strawberry_wrist_attention_localization \
    -p use_sim_time:=true \
    -p confidence_threshold:=0.58 \
    -p detections_topic:=/strawberry/shadow/detections \
    -p target_pose_topic:=/strawberry/shadow/target_pose \
    -p depth_topic:=/camera/wrist/depth/image_raw \
    -p camera_info_topic:=/camera/wrist/camera_info \
    -p sensor_qos_depth:=30 \
    -p selection_roi_min_x_px:="${roi_min_x}" \
    -p selection_roi_min_y_px:="${roi_min_y}" \
    -p selection_roi_max_x_px:="${roi_max_x}" \
    -p selection_roi_max_y_px:="${roi_max_y}" \
    -p allow_stationary_latest_tf_fallback:=true \
    -p allow_stationary_sensor_sync_fallback:=true \
    -p stationary_sync_tolerance_sec:=0.105 \
    >"${output_dir}/wrist_localization.log" 2>&1 &
  pipeline_pids+=("$!")
}

setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true camera_mount:=dual enable_attachment:=false \
  >"${output_dir}/sim.log" 2>&1 &
sim_pid=$!

start_base_pipeline
timeout --signal=TERM 240 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/base_window.json" \
  --scenario-id blender_v2_dual_base_overview_same_world \
  --lighting nominal --occlusion natural_plant \
  --warmup-frames 10 --frames 60 --timeout-sec 180 \
  --post-window-wait-sec 2 \
  --window-boundary base_overview_before_robot_motion \
  >"${output_dir}/base_window.log" 2>&1
python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/base/color/image_raw \
  --output "${output_dir}/base_before_motion_rgb.png" --timeout-sec 30 \
  >"${output_dir}/base_capture.log" 2>&1

ros2 run strawberry_bringup dual_observation_selector \
  --input-json "${output_dir}/base_window.json" \
  --output-json "${output_dir}/selection.json" \
  >"${output_dir}/selection.log" 2>&1

selection_values="$(
  python -c \
    'import json,sys; d=json.load(open(sys.argv[1])); print(d["candidate_target_id"], d["selected_preset"], *d["focus_roi_xyxy_px"])' \
    "${output_dir}/selection.json"
)"
read -r expected_target_id selected_preset roi_min_x roi_min_y roi_max_x roi_max_y \
  <<<"${selection_values}"

stop_pipeline

pose_args=()
case "${selected_preset}" in
  center)
    ;;
  lower)
    pose_args=(
      --qx 0.0
      --qy 0.9537169507
      --qz 0.0
      --qw 0.3007057995
    )
    ;;
  *)
    echo "Selector returned unsupported preset: ${selected_preset}" >&2
    exit 1
    ;;
esac

timeout --signal=TERM 120 python \
  "${repo_root}/scripts/move_wrist_observation_pose.py" \
  --camera-mount dual \
  --planning-attempts 3 \
  "${pose_args[@]}" \
  --output "${output_dir}/observation_motion.json" \
  >"${output_dir}/observation_motion.log" 2>&1

start_wrist_pipeline \
  "${roi_min_x}" "${roi_min_y}" "${roi_max_x}" "${roi_max_y}"
timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/wrist_window.json" \
  --scenario-id "blender_v2_dual_wrist_${selected_preset}_same_world" \
  --lighting nominal --occlusion natural_plant \
  --warmup-frames 0 --ready-consecutive-target-pose-frames 15 \
  --frames 60 --timeout-sec 120 \
  --post-window-wait-sec 2 \
  --window-boundary after_wrist_observation_settle_before_pick_motion \
  >"${output_dir}/wrist_window.log" 2>&1
ros2 node list --no-daemon \
  | grep -Ev '^/_ros2cli_[[:alnum:]_]+$' \
  | sort -u >"${output_dir}/wrist_runtime_nodes.txt"
ros2 topic list --no-daemon \
  | sort -u >"${output_dir}/wrist_runtime_topics.txt"
# The just-finished wrist receipt is the authoritative target-topic proof.
# A fresh ``ros2 topic list --no-daemon`` process can see an incomplete graph
# while DDS discovery converges, so its inventory is retained as diagnostic
# evidence but is not required to rediscover the known publisher immediately.
if grep -Fxq "/strawberry/oracle/target_pose" \
  "${output_dir}/wrist_runtime_topics.txt"; then
  echo "Oracle target topic unexpectedly exists" >&2
  exit 1
fi
if grep -Eqi "oracle|orchestrator" "${output_dir}/wrist_runtime_nodes.txt"; then
  echo "Oracle or orchestrator node unexpectedly exists" >&2
  exit 1
fi
timeout --signal=TERM 180 ros2 run strawberry_manipulation \
  handoff_shadow_probe \
  --output-json "${output_dir}/handoff_shadow.json" \
  --expected-target-id "${expected_target_id}" \
  --target-topic /strawberry/shadow/target_pose \
  --camera-mount dual \
  --timeout-sec 60 \
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
  --target-topic /strawberry/shadow/target_pose \
  --camera-mount dual \
  --timeout-sec 60 \
  --planning-attempts 3 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/pregrasp_shadow.log" 2>&1
python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/wrist/color/image_raw \
  --output "${output_dir}/wrist_observation_rgb.png" --timeout-sec 30 \
  >"${output_dir}/wrist_capture.log" 2>&1
python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/base/color/image_raw \
  --output "${output_dir}/base_after_motion_rgb.png" --timeout-sec 30 \
  >"${output_dir}/base_after_capture.log" 2>&1
python "${repo_root}/scripts/capture_tf_transform.py" \
  --source-frame panda_link0 \
  --target-frame strawberry_wrist_camera_optical_frame \
  --output "${output_dir}/wrist_camera_tf.json" \
  >"${output_dir}/wrist_camera_tf.log" 2>&1

stop_pipeline
stop_group "${sim_pid}"
sim_pid=""

ros2 run strawberry_bringup dual_observation_summary \
  --selection-json "${output_dir}/selection.json" \
  --motion-json "${output_dir}/observation_motion.json" \
  --wrist-window-json "${output_dir}/wrist_window.json" \
  --handoff-shadow-json "${output_dir}/handoff_shadow.json" \
  --pregrasp-shadow-json "${output_dir}/pregrasp_shadow.json" \
  --output-json "${output_dir}/sequence_summary.json" \
  >"${output_dir}/sequence_summary.log" 2>&1

trap - EXIT INT TERM
if grep -Eqi \
  "Traceback|process has died|RuntimeError|exception was never retrieved" \
  "${output_dir}/sim.log" \
  "${output_dir}/base_perception.log" \
  "${output_dir}/base_localization.log" \
  "${output_dir}/base_window.log" \
  "${output_dir}/observation_motion.log" \
  "${output_dir}/wrist_perception.log" \
  "${output_dir}/wrist_localization.log" \
  "${output_dir}/wrist_window.log" \
  "${output_dir}/handoff_shadow.log" \
  "${output_dir}/pregrasp_shadow.log"; then
  echo "Sequential observation logs contain an unhandled failure" >&2
  exit 1
fi
if grep -Eqi "Added FollowJointTrajectory|Added GripperCommand" \
  "${output_dir}/handoff_shadow.log" \
  "${output_dir}/pregrasp_shadow.log"; then
  echo "Control-side Shadow unexpectedly loaded a motion controller" >&2
  exit 1
fi
echo "Dual sequential observation: ${output_dir}/sequence_summary.json"
