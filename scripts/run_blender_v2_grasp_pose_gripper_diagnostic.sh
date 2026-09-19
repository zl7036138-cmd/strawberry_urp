#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/development/blender_v2_grasp_pose_gripper_diagnostic_v1}"
readiness="${2:-${repo_root}/results/development/blender_v2_perception_pick_once_v3/execution_readiness.json}"
domain_id="${3:-228}"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ -f "${readiness}" ]] || {
  echo "Perception execution readiness is missing: ${readiness}" >&2
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
stop_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 300); do
    kill -0 -- "-${launch_pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap stop_launch EXIT INT TERM

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  camera_mount:=dual \
  enable_attachment:=false \
  enable_pose_control:=false \
  start_perception:=false \
  start_oracle_provider:=false \
  start_manipulation:=false \
  start_orchestrator:=false \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

set +e
timeout --signal=TERM 240 python \
  "${repo_root}/scripts/run_blender_v2_grasp_pose_gripper_diagnostic.py" \
  --readiness "${readiness}" \
  --output "${output_dir}/summary.json" \
  >"${output_dir}/probe.log" 2>&1
probe_status=$?
set -e

stop_launch
trap - EXIT INT TERM
cat "${output_dir}/probe.log"
exit "${probe_status}"
