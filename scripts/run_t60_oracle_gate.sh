#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
trial_count="${1:-10}"
base_domain_id="${2:-140}"
output_dir="${3:-${repo_root}/results/t60/oracle_gate}"
extra_launch_arguments=("${@:4}")
expect_shadow=false
for argument in "${extra_launch_arguments[@]}"; do
  if [[ "${argument}" == "shadow_enabled:=true" ]]; then
    expect_shadow=true
  fi
done

if ! [[ "${trial_count}" =~ ^[1-9][0-9]*$ ]]; then
  echo "trial_count must be a positive integer" >&2
  exit 2
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  echo "base_domain_id must be a non-negative integer" >&2
  exit 2
fi
if ((base_domain_id + trial_count - 1 > 232)); then
  echo "requested ROS domain IDs exceed the supported maximum of 232" >&2
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
  client_log="${output_dir}/${trial_label}_client.log"
  result_json="${output_dir}/${trial_label}.json"
  echo "[${trial_index}/${trial_count}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true \
    start_oracle_provider:=true \
    start_manipulation:=true \
    start_orchestrator:=true \
    enable_attachment:=true \
    target_source:=oracle \
    control_target_topic:=/strawberry/oracle/target_pose \
    "${extra_launch_arguments[@]}" \
    >"${launch_log}" 2>&1 &
  launch_pid=$!

  client_arguments=(
    --startup-timeout-sec 60
    --trial-timeout-sec 180
    --output "${result_json}"
  )
  if [[ "${expect_shadow}" == "true" ]]; then
    client_arguments+=(--expect-shadow)
  fi
  if python "${repo_root}/scripts/test_orchestrated_trial.py" \
    "${client_arguments[@]}" \
    >"${client_log}" 2>&1; then
    client_status=0
  else
    client_status=$?
  fi
  echo "[${trial_index}/${trial_count}] client status ${client_status}"
  shutdown_launch
done

trap - EXIT INT TERM
python "${repo_root}/scripts/summarize_t60_oracle_gate.py" \
  --output-dir "${output_dir}" \
  --expected-trials "${trial_count}" \
  --base-domain-id "${base_domain_id}"
