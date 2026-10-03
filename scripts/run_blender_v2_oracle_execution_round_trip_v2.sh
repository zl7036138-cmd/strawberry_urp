#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
contract="${repo_root}/config/blender_v2_oracle_execution_round_trip_v2.json"
output_dir="${repo_root}/results/development/blender_v2_oracle_execution_round_trip_v2"
preflight_json="${output_dir}/initial_state.json"
trial_json="${output_dir}/trial_01.json"
summary_json="${output_dir}/summary.json"
launch_log="${output_dir}/launch.log"

[[ ! -e "${output_dir}" ]] || {
  echo "refusing to overwrite frozen output: ${output_dir}" >&2
  exit 2
}
[[ -f "${contract}" ]] || {
  echo "frozen v2 execution contract is missing: ${contract}" >&2
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

for candidate in $(seq 190 232); do
  export ROS_DOMAIN_ID="${candidate}"
  if ! ros2 node list --no-daemon --all --spin-time 1 2>/dev/null | grep -q .; then
    break
  fi
done

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  camera_mount:=dual \
  start_perception:=false \
  start_oracle_provider:=false \
  start_manipulation:=true \
  start_orchestrator:=false \
  enable_attachment:=true \
  enable_pose_control:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

set +e
timeout --signal=TERM 90 python \
  "${repo_root}/scripts/capture_blender_v2_execution_initial_state.py" \
  --output "${preflight_json}" \
  --target-id 1 \
  --required-samples 10 \
  --maximum-ready-error-rad 0.02 \
  --timeout-sec 60 \
  >"${output_dir}/preflight.log" 2>&1
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
    --output "${trial_json}" \
    >"${output_dir}/client.log" 2>&1
  client_status=$?
  set -e
else
  client_status=125
fi

shutdown_launch
trap - EXIT INT TERM

if [[ ! -f "${preflight_json}" ]]; then
  printf '{"passed":false,"violations":["preflight produced no result"]}\n' \
    >"${preflight_json}"
fi
if [[ ! -f "${trial_json}" ]]; then
  printf '{"success":false,"error":"action not run or client status %s"}\n' \
    "${client_status}" >"${trial_json}"
fi

python "${repo_root}/scripts/validate_blender_v2_oracle_execution_v2.py" \
  --contract "${contract}" \
  --preflight "${preflight_json}" \
  --trial "${trial_json}" \
  --launch-log "${launch_log}" \
  --output "${summary_json}" \
  --repository-root "${repo_root}"
