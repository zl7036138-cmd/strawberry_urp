#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runner_script="$(realpath "${BASH_SOURCE[0]}")"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t60/sim_perception_pre_gate_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640/weights/best.pt}"
domain_id="${3:-225}"
frames_per_scenario="${4:-10}"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if [[ ! -f "${model_path}" ]]; then
  echo "Model does not exist: ${model_path}" >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi
if ! [[ "${domain_id}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]]; then
  echo "ROS domain ID must be an integer from 0 through 232" >&2
  exit 2
fi
if ! [[ "${frames_per_scenario}" =~ ^[1-9][0-9]*$ ]]; then
  echo "frames_per_scenario must be a positive integer" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"

domain_nodes="$({ timeout 5 ros2 node list --no-daemon --spin-time 1.0 --all || true; } | grep -Ev '^/?_ros2cli_[0-9]+$' || true)"
if [[ -n "${domain_nodes}" ]]; then
  echo "ROS domain ${domain_id} is not empty:" >&2
  echo "${domain_nodes}" >&2
  exit 5
fi

mkdir -p "${output_dir}"
launch_log="${output_dir}/launch.log"
gate_log="${output_dir}/gate.log"
summary_json="${output_dir}/summary.json"
frames_csv="${output_dir}/frames.csv"
launch_pid=""

shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
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

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  enable_pose_control:=true \
  start_perception:=true \
  model_path:="${model_path}" \
  confidence_threshold:=0.31 \
  >"${launch_log}" 2>&1 &
launch_pid=$!

gate_status=0
timeout --signal=TERM 240 ros2 run strawberry_perception sim_perception_gate \
  --output-json "${summary_json}" \
  --output-csv "${frames_csv}" \
  --model-path "${model_path}" \
  --runner-script "${runner_script}" \
  --frames-per-scenario "${frames_per_scenario}" \
  >"${gate_log}" 2>&1 || gate_status=$?

shutdown_launch
trap - EXIT INT TERM

if [[ ! -s "${summary_json}" ]] || [[ ! -s "${frames_csv}" ]]; then
  echo "Simulator perception pre-gate did not produce complete artifacts." >&2
  gate_status=1
fi
if grep -Eqi "Traceback|exception was never retrieved|process has died" \
  "${launch_log}" "${gate_log}"; then
  echo "Simulator perception pre-gate logs contain an unhandled runtime failure." >&2
  gate_status=1
fi

echo "Simulator perception pre-gate: ${summary_json}"
echo "Frame records: ${frames_csv}"
echo "ROS domain: ${ROS_DOMAIN_ID}"
exit "${gate_status}"
