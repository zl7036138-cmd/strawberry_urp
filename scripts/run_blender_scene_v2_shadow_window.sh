#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/blender_scene_v2_shadow_60f_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
domain_id="${3:-225}"

expected_model_sha256="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
confidence_threshold="0.58"
warmup_frames="10"
measurement_frames="60"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${artifact_root}/install/setup.bash" >&2
  exit 3
fi
if [[ ! -f "${model_path}" ]]; then
  echo "Model does not exist: ${model_path}" >&2
  exit 2
fi
actual_model_sha256="$(sha256sum "${model_path}" | awk '{print $1}')"
if [[ "${actual_model_sha256}" != "${expected_model_sha256}" ]]; then
  echo "Model hash does not match the engineering waiver." >&2
  exit 2
fi
if [[ -e "${output_dir}" ]]; then
  echo "Output already exists; choose a fresh evidence directory: ${output_dir}" >&2
  exit 4
fi
if ! [[ "${domain_id}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]]; then
  echo "ROS domain ID must be in [0, 232]." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"

domain_nodes="$({
  timeout 5 ros2 node list --no-daemon --spin-time 1.0 --all || true
} | grep -Ev '^/?_ros2cli_[0-9]+$' || true)"
if [[ -n "${domain_nodes}" ]]; then
  echo "ROS domain ${domain_id} is not empty:" >&2
  echo "${domain_nodes}" >&2
  exit 5
fi

mkdir -p "${output_dir}"
launch_log="${output_dir}/launch.log"
window_log="${output_dir}/shadow_window.log"
window_json="${output_dir}/shadow_window.json"
launch_pid=""

shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi
  kill -TERM "${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  start_perception:=true \
  start_oracle_provider:=false \
  start_manipulation:=false \
  start_orchestrator:=false \
  enable_attachment:=false \
  enable_pose_control:=false \
  model_path:="${model_path}" \
  confidence_threshold:="${confidence_threshold}" \
  perception_detections_topic:=/strawberry/shadow/detections \
  perception_target_topic:=/strawberry/shadow/target_pose \
  >"${launch_log}" 2>&1 &
launch_pid=$!

window_status=0
timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe \
  --output-json "${window_json}" \
  --scenario-id blender_scene_v2_nominal_fixed_camera \
  --lighting nominal \
  --occlusion natural_plant \
  --frames "${measurement_frames}" \
  --warmup-frames "${warmup_frames}" \
  --window-boundary after_model_warmup_before_robot_motion \
  --timeout-sec 120 \
  --post-window-wait-sec 2.0 \
  >"${window_log}" 2>&1 || window_status=$?

shutdown_launch
trap - EXIT INT TERM

if [[ "${window_status}" -ne 0 ]] || [[ ! -s "${window_json}" ]]; then
  echo "Shadow window failed with status ${window_status}." >&2
  exit 1
fi
if grep -Eqi "Traceback|exception was never retrieved|process has died" \
  "${launch_log}" "${window_log}"; then
  echo "Shadow logs contain an unhandled runtime failure." >&2
  exit 1
fi

echo "Shadow window: ${window_json}"
echo "Model SHA-256: ${actual_model_sha256}"
echo "Confidence threshold: ${confidence_threshold}"
