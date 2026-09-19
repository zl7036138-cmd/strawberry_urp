#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
contract="${repo_root}/config/blender_v2_gripper_contact_round_trip_v1.json"
output_dir="${repo_root}/results/development/blender_v2_gripper_contact_round_trip_v1"
result_json="${output_dir}/summary.json"
launch_log="${output_dir}/launch.log"
probe_log="${output_dir}/probe.log"

if [[ -e "${output_dir}" ]]; then
  echo "refusing to overwrite frozen output: ${output_dir}" >&2
  exit 2
fi
if [[ ! -f "${contract}" ]]; then
  echo "missing frozen contract: ${contract}" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set -u
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "workspace is not built: ${artifact_root}/install/setup.bash" >&2
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
  for _ in $(seq 1 200); do
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

for candidate in $(seq 170 232); do
  export ROS_DOMAIN_ID="${candidate}"
  if ! ros2 node list --no-daemon --all --spin-time 1 2>/dev/null | grep -q .; then
    break
  fi
done

setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true \
  camera_mount:=dual \
  enable_attachment:=true \
  enable_pose_control:=true \
  >"${launch_log}" 2>&1 &
launch_pid=$!

set +e
python "${repo_root}/scripts/run_blender_v2_gripper_contact_trial.py" \
  --contract "${contract}" \
  --output "${result_json}" \
  >"${probe_log}" 2>&1
probe_status=$?
set -e

shutdown_launch
trap - EXIT INT TERM
cat "${probe_log}"
exit "${probe_status}"
