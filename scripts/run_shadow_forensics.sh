#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
model_path="${1:-${repo_root}/outputs/perception/yolo11s_640/weights/best.pt}"
output_dir="${2:-${repo_root}/results/t60/shadow_forensics_rgb_bgr_v1}"
domain_id="${3:-222}"
frame_count="${4:-5}"

if [[ ! -f "${model_path}" ]]; then
  echo "Model does not exist: ${model_path}" >&2
  exit 2
fi
if [[ -e "${output_dir}" ]]; then
  echo "Output directory already exists; choose a new evidence path: ${output_dir}" >&2
  exit 2
fi
if ! [[ "${domain_id}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]]; then
  echo "ROS domain ID must be an integer from 0 through 232" >&2
  exit 2
fi
if ! [[ "${frame_count}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Frame count must be a positive integer" >&2
  exit 2
fi
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"

launch_pid=""
shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

launch_log="${output_dir}_launch.log"
setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true \
  enable_attachment:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

sleep 10
timeout 180 ros2 run strawberry_perception shadow_diagnostic --ros-args \
  -p "model_path:=${model_path}" \
  -p "output_dir:=${output_dir}" \
  -p "frame_count:=${frame_count}" \
  -p sample_period_sec:=0.5 \
  -p confidence_threshold:=0.31

test -s "${output_dir}/summary.json"
echo "Shadow forensic evidence: ${output_dir}/summary.json"
