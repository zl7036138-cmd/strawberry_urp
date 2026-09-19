#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
contract="${repo_root}/config/blender_v2_oracle_execution_repeat_5_v2.json"
output_dir="${repo_root}/results/development/blender_v2_oracle_execution_repeat_5_v2"
trial_count=5

[[ ! -e "${output_dir}" ]] || {
  echo "refusing to overwrite frozen output: ${output_dir}" >&2
  exit 2
}
[[ -f "${contract}" ]] || {
  echo "frozen repeat-v2 contract is missing: ${contract}" >&2
  exit 2
}
[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "workspace is not built: ${artifact_root}" >&2
  exit 3
}

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
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
  for _ in $(seq 1 300); do
    kill -0 -- "-${launch_pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

base_domain_id=""
for candidate in $(seq 200 228); do
  range_free=true
  for offset in $(seq 0 $((trial_count - 1))); do
    export ROS_DOMAIN_ID=$((candidate + offset))
    discovered="$(
      ros2 node list --no-daemon --all --spin-time 1 2>/dev/null \
        | grep -Ev '^/_ros2cli_[0-9]+$' || true
    )"
    if [[ -n "${discovered}" ]]; then
      range_free=false
      break
    fi
  done
  if [[ "${range_free}" == "true" ]]; then
    base_domain_id="${candidate}"
    break
  fi
done
[[ -n "${base_domain_id}" ]] || {
  echo "no free five-domain range is available" >&2
  exit 4
}

for trial_index in $(seq 1 "${trial_count}"); do
  printf -v trial_label "trial_%02d" "${trial_index}"
  trial_dir="${output_dir}/${trial_label}"
  mkdir -p "${trial_dir}"
  export ROS_DOMAIN_ID=$((base_domain_id + trial_index - 1))
  echo "[${trial_index}/${trial_count}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true \
    camera_mount:=dual \
    start_perception:=false \
    start_oracle_provider:=false \
    start_manipulation:=true \
    start_orchestrator:=false \
    enable_attachment:=true \
    enable_pose_control:=false \
    >"${trial_dir}/launch.log" 2>&1 &
  launch_pid=$!

  set +e
  timeout --signal=TERM 90 python \
    "${repo_root}/scripts/capture_blender_v2_execution_initial_state.py" \
    --output "${trial_dir}/initial_state.json" \
    --target-id 1 \
    --required-samples 10 \
    --maximum-ready-error-rad 0.02 \
    --timeout-sec 60 \
    >"${trial_dir}/preflight.log" 2>&1
  preflight_status=$?
  set -e

  if [[ "${preflight_status}" -eq 0 ]]; then
    set +e
    timeout --signal=TERM 240 python \
      "${repo_root}/scripts/test_oracle_pick_and_place.py" \
      --target-id 1 \
      --target-topic /strawberry/ground_truth/fruit_1/pose \
      --place-x 0.35 \
      --place-y -0.45 \
      --place-z 0.45 \
      --startup-timeout-sec 60 \
      --action-timeout-sec 180 \
      --stability-samples 10 \
      --stability-tolerance-m 0.001 \
      --output "${trial_dir}/trial.json" \
      >"${trial_dir}/client.log" 2>&1
    client_status=$?
    set -e
  else
    client_status=125
  fi

  shutdown_launch
  if [[ ! -f "${trial_dir}/initial_state.json" ]]; then
    printf '{"passed":false,"violations":["preflight produced no result"]}\n' \
      >"${trial_dir}/initial_state.json"
  fi
  if [[ ! -f "${trial_dir}/trial.json" ]]; then
    printf '{"success":false,"error":"action not run or client status %s"}\n' \
      "${client_status}" >"${trial_dir}/trial.json"
  fi
done

trap - EXIT INT TERM
python "${repo_root}/scripts/summarize_blender_v2_oracle_execution_repeat_v2.py" \
  --contract "${contract}" \
  --results "${output_dir}" \
  --output "${output_dir}/summary.json" \
  --repository-root "${repo_root}"
