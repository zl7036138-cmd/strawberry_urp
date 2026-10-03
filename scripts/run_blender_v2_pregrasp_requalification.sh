#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
contract="${repo_root}/config/blender_v2_pregrasp_planning_post_contact_v1.json"
handoff="${repo_root}/artifacts/simulation/blender_v2_pregrasp_oracle_handoff_v1.json"
output_dir="${repo_root}/results/development/blender_v2_pregrasp_planning_post_contact_v1"
pregrasp_json="${output_dir}/pregrasp_shadow.json"
summary_json="${output_dir}/summary.json"
launch_log="${output_dir}/launch.log"
pregrasp_log="${output_dir}/pregrasp.log"

[[ ! -e "${output_dir}" ]] || {
  echo "refusing to overwrite frozen output: ${output_dir}" >&2
  exit 2
}
[[ -f "${contract}" && -f "${handoff}" ]] || {
  echo "frozen contract or handoff is missing" >&2
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
  for _ in $(seq 1 200); do
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

for candidate in $(seq 170 232); do
  export ROS_DOMAIN_ID="${candidate}"
  if ! ros2 node list --no-daemon --all --spin-time 1 2>/dev/null | grep -q .; then
    break
  fi
done

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=true \
  camera_mount:=dual \
  start_oracle_provider:=true \
  start_perception:=false \
  start_manipulation:=false \
  start_orchestrator:=false \
  enable_attachment:=false \
  enable_pose_control:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

set +e
timeout --signal=TERM 180 ros2 run strawberry_manipulation \
  pregrasp_planning_shadow \
  --handoff-json "${handoff}" \
  --output-json "${pregrasp_json}" \
  --target-topic /strawberry/oracle/target_pose \
  --camera-mount dual \
  --timeout-sec 60 \
  --planning-attempts 3 \
  --minimum-target-samples 15 \
  --minimum-joint-samples 10 \
  --maximum-target-age-sec 0.5 \
  --maximum-joint-delta-rad 0.002 \
  --ros-args -p use_sim_time:=true \
  >"${pregrasp_log}" 2>&1
pregrasp_status=$?
set -e

shutdown_launch
trap - EXIT INT TERM
if [[ "${pregrasp_status}" -ne 0 ]]; then
  cat "${pregrasp_log}" >&2
  exit "${pregrasp_status}"
fi

python "${repo_root}/scripts/validate_blender_v2_pregrasp_requalification.py" \
  --contract "${contract}" \
  --pregrasp "${pregrasp_json}" \
  --output "${summary_json}" \
  --repository-root "${repo_root}"
