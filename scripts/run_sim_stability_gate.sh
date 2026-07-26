#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
duration_sec="${1:-1800}"
enable_attachment="${ENABLE_ATTACHMENT:-true}"
output_dir="${repo_root}/results/p0"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
launch_log="${output_dir}/sim_stability_${timestamp}.log"
health_json="${output_dir}/sim_stability_${timestamp}.json"

source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set -u
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${repo_root}/scripts/build_and_test.sh first." >&2
  exit 3
fi
set +u
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
mkdir -p "${output_dir}"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    # Bash starts asynchronous jobs with SIGINT ignored.  Put launch and all
    # children in their own session.  SIGTERM the launch process first so it
    # can stop and reap its children; the exact process group is the bounded
    # fallback if a child does not honour launch's shutdown request.
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
  fi
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true \
  enable_attachment:="${enable_attachment}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

health_status=0
ros2 run strawberry_sim runtime_health_check \
  --duration-sec "${duration_sec}" \
  --output "${health_json}" || health_status=$?

if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "Simulation launch exited before the gate completed." >&2
  health_status=1
fi

shutdown_launch
trap - EXIT INT TERM

echo "Runtime result: ${health_json}"
echo "Launch log: ${launch_log}"
exit "${health_status}"
