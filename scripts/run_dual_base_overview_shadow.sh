#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/dual_base_overview_shadow_60f_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-225}"
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
  headless:=true camera_mount:=dual start_perception:=true \
  start_oracle_provider:=false start_manipulation:=false start_orchestrator:=false \
  enable_attachment:=false enable_pose_control:=false model_path:="${model_path}" \
  confidence_threshold:=0.58 \
  perception_image_topic:=/camera/base/color/image_raw \
  localization_depth_topic:=/camera/base/depth/image_raw \
  localization_camera_info_topic:=/camera/base/camera_info \
  perception_detections_topic:=/strawberry/shadow/detections \
  perception_target_topic:=/strawberry/shadow/target_pose \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

timeout --signal=TERM 240 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${output_dir}/shadow_window.json" \
  --scenario-id blender_v2_dual_base_overview \
  --lighting nominal --occlusion natural_plant \
  --warmup-frames 10 --frames 60 --timeout-sec 180 \
  --post-window-wait-sec 2 \
  --window-boundary base_overview_before_robot_motion \
  >"${output_dir}/shadow_window.log" 2>&1

python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/base/color/image_raw \
  --output "${output_dir}/overview_rgb.png" --timeout-sec 30 \
  >"${output_dir}/capture.log" 2>&1
python "${repo_root}/scripts/capture_tf_transform.py" \
  --source-frame panda_link0 \
  --target-frame strawberry_base_camera_optical_frame \
  --output "${output_dir}/camera_tf.json" \
  >"${output_dir}/camera_tf.log" 2>&1

shutdown_launch
trap - EXIT INT TERM
if grep -Eqi "Traceback|process has died|RuntimeError" \
  "${output_dir}/launch.log" "${output_dir}/shadow_window.log"; then
  echo "Dual base-overview logs contain an unhandled failure" >&2
  exit 1
fi
echo "Dual base overview Shadow: ${output_dir}/shadow_window.json"
