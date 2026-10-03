#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
domain_id="${1:-229}"
output_dir="${2:-${repo_root}/artifacts/sim/field_v3_headed_perception_demo}"
model_path="${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt"
expected_model_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ -f "${model_path}" ]] || {
  echo "Accepted model is missing: ${model_path}" >&2
  exit 2
}
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_model_sha}" ]] || {
  echo "Model hash does not match the accepted runtime checkpoint" >&2
  exit 2
}

mkdir -p "${output_dir}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"

if ! timeout 10 ros2 topic list \
  | grep -qx "/camera/wrist/color/image_raw"; then
  echo "The headed field-v3 simulation is not available in domain ${domain_id}" >&2
  exit 4
fi

children=()
cleanup() {
  local child
  for child in "${children[@]}"; do
    kill -TERM -- "-${child}" 2>/dev/null || true
  done
  for child in "${children[@]}"; do
    wait "${child}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid ros2 run strawberry_perception perception_node \
  --ros-args \
  -p use_sim_time:=true \
  -p model_path:="${model_path}" \
  -p image_topic:=/camera/wrist/color/image_raw \
  -p detections_topic:=/strawberry/demo/detections \
  -p confidence_threshold:=0.31 \
  -p image_size:=640 \
  -p nms_iou_threshold:=0.70 \
  >"${output_dir}/perception.log" 2>&1 &
children+=("$!")

setsid ros2 run strawberry_localization localization_node \
  --ros-args \
  --params-file \
  "${repo_root}/ros2_ws/src/strawberry_localization/config/localization_blender_v2.yaml" \
  -p use_sim_time:=true \
  -p detections_topic:=/strawberry/demo/detections \
  -p target_pose_topic:=/strawberry/demo/target_pose \
  -p depth_topic:=/camera/wrist/depth/image_raw \
  -p camera_info_topic:=/camera/wrist/camera_info \
  -p confidence_threshold:=0.31 \
  -p surface_to_center_offset_m:=0.026 \
  -p selection_roi_min_x_px:=320 \
  -p selection_roi_min_y_px:=240 \
  -p selection_roi_max_x_px:=640 \
  -p selection_roi_max_y_px:=480 \
  >"${output_dir}/localization.log" 2>&1 &
children+=("$!")

python "${repo_root}/scripts/show_field_v3_perception.py" \
  --image-topic /camera/wrist/color/image_raw \
  --detections-topic /strawberry/demo/detections \
  --target-topic /strawberry/demo/target_pose \
  --roi 320 240 640 480 \
  --ros-args -p use_sim_time:=true \
  >"${output_dir}/display.log" 2>&1
