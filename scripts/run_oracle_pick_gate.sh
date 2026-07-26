#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
trial_count="${1:-10}"
base_domain_id="${2:-100}"
output_dir="${3:-${repo_root}/results/p2/oracle_gate}"
camera_mount="${STRAWBERRY_CAMERA_MOUNT:-fixed}"

if ! [[ "${trial_count}" =~ ^[1-9][0-9]*$ ]]; then
  echo "trial_count must be a positive integer" >&2
  exit 2
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  echo "base_domain_id must be a non-negative integer" >&2
  exit 2
fi
last_domain_id=$((base_domain_id + trial_count - 1))
if ((last_domain_id > 232)); then
  echo "requested ROS domain IDs exceed the supported maximum of 232" >&2
  exit 2
fi
if [[ "${camera_mount}" != "fixed" && "${camera_mount}" != "wrist" && "${camera_mount}" != "dual" ]]; then
  echo "STRAWBERRY_CAMERA_MOUNT must be fixed, wrist, or dual" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set -u
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
set +u
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_dir}"

launch_pid=""
shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi

  # Background jobs inherit SIGINT ignored from Bash. Signal the launch
  # process with SIGTERM so it can stop and reap every child. The separate
  # process group is only a bounded fallback for a stuck process.
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

for trial_index in $(seq 1 "${trial_count}"); do
  printf -v trial_label "trial_%02d" "${trial_index}"
  export ROS_DOMAIN_ID=$((base_domain_id + trial_index - 1))
  launch_log="${output_dir}/${trial_label}_launch.log"
  action_log="${output_dir}/${trial_label}_action.log"
  result_json="${output_dir}/${trial_label}.json"

  echo "[${trial_index}/${trial_count}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true \
    camera_mount:="${camera_mount}" \
    start_manipulation:=true \
    enable_attachment:=true \
    >"${launch_log}" 2>&1 &
  launch_pid=$!

  if python "${repo_root}/scripts/test_oracle_pick_and_place.py" \
    --startup-timeout-sec 60 \
    --action-timeout-sec 180 \
    --output "${result_json}" \
    >"${action_log}" 2>&1; then
    action_status=0
  else
    action_status=$?
  fi

  if [[ "${action_status}" -eq 0 ]]; then
    echo "[${trial_index}/${trial_count}] action succeeded"
  else
    echo "[${trial_index}/${trial_count}] action failed with status ${action_status}"
  fi
  shutdown_launch
done

trap - EXIT INT TERM

python "${repo_root}/scripts/summarize_oracle_gate.py" \
  --output-dir "${output_dir}" \
  --expected-trials "${trial_count}"
