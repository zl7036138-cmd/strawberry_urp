#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/wrist_observation_shadow_60f_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-231}"
camera_mount="${4:-wrist}"
observation_pose="${5:-center}"
expected_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
[[ -f "${artifact_root}/install/setup.bash" ]] || { echo "Workspace is not built" >&2; exit 3; }
[[ -f "${model_path}" ]] || { echo "Model missing" >&2; exit 2; }
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_sha}" ]] || { echo "Wrong model hash" >&2; exit 2; }
[[ ! -e "${output_dir}" ]] || { echo "Output exists; choose a fresh directory" >&2; exit 4; }
[[ "${camera_mount}" == "wrist" || "${camera_mount}" == "dual" ]] || {
  echo "Camera mount must be wrist or dual" >&2
  exit 2
}
case "${observation_pose}" in
  center)
    scenario_id="blender_v2_wrist_observation_center"
    pose_args=()
    selection_roi_launch_args=()
    ;;
  lower)
    scenario_id="blender_v2_wrist_observation_lower"
    pose_args=(
      --qx 0.0
      --qy 0.9537169507
      --qz 0.0
      --qw 0.3007057995
    )
    selection_roi_launch_args=(
      localization_selection_roi_min_x_px:=320
      localization_selection_roi_min_y_px:=240
      localization_selection_roi_max_x_px:=640
      localization_selection_roi_max_y_px:=480
    )
    ;;
  *)
    echo "Observation pose must be center or lower" >&2
    exit 2
    ;;
esac
if [[ "${camera_mount}" == "dual" ]]; then
  image_topic="/camera/wrist/color/image_raw"
  depth_topic="/camera/wrist/depth/image_raw"
  camera_info_topic="/camera/wrist/camera_info"
  camera_frame="strawberry_wrist_camera_optical_frame"
else
  image_topic="/camera/color/image_raw"
  depth_topic="/camera/depth/image_raw"
  camera_info_topic="/camera/camera_info"
  camera_frame="strawberry_camera_optical_frame"
fi
set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"
mkdir -p "${output_dir}"
launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 200); do kill -0 -- "-${launch_pid}" 2>/dev/null || break; sleep .1; done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM
setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true camera_mount:="${camera_mount}" start_perception:=true \
  start_oracle_provider:=false start_manipulation:=false start_orchestrator:=false \
  enable_attachment:=false enable_pose_control:=false model_path:="${model_path}" \
  confidence_threshold:=0.58 \
  allow_stationary_latest_tf_fallback:=true \
  perception_image_topic:="${image_topic}" \
  localization_depth_topic:="${depth_topic}" \
  localization_camera_info_topic:="${camera_info_topic}" \
  "${selection_roi_launch_args[@]}" \
  perception_detections_topic:=/strawberry/shadow/detections \
  perception_target_topic:=/strawberry/shadow/target_pose \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!
timeout --signal=TERM 120 python "${repo_root}/scripts/move_wrist_observation_pose.py" \
  --camera-mount "${camera_mount}" \
  "${pose_args[@]}" \
  --output "${output_dir}/observation_motion.json" \
  >"${output_dir}/observation_motion.log" 2>&1
timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/shadow_window.json" \
  --scenario-id "${scenario_id}" \
  --lighting nominal --occlusion natural_plant \
  --warmup-frames 10 --frames 60 --timeout-sec 120 \
  --post-window-wait-sec 2 \
  --window-boundary after_wrist_observation_settle_before_pick_motion \
  >"${output_dir}/shadow_window.log" 2>&1
python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic "${image_topic}" \
  --output "${output_dir}/observation_rgb.png" --timeout-sec 30 \
  >"${output_dir}/capture.log" 2>&1
python "${repo_root}/scripts/capture_tf_transform.py" \
  --source-frame panda_link0 --target-frame "${camera_frame}" \
  --output "${output_dir}/camera_tf.json" \
  >"${output_dir}/camera_tf.log" 2>&1
shutdown_launch
trap - EXIT INT TERM
if grep -Eqi "Traceback|process has died|RuntimeError" \
  "${output_dir}/launch.log" "${output_dir}/observation_motion.log" \
  "${output_dir}/shadow_window.log"; then
  echo "Wrist observation Shadow logs contain an unhandled failure" >&2
  exit 1
fi
echo "Wrist observation Shadow: ${output_dir}/shadow_window.json"
