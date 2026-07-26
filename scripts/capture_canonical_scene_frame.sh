#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
colcon_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output="${1:-${repo_root}/artifacts/sim/blend_inspection/canonical_plant_scene.png}"
launch_log="${output%.png}_launch.log"

source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set +u
source "${colcon_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
mkdir -p "$(dirname "${output}")"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 100); do
      if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true \
  enable_attachment:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

python "${repo_root}/tools/ros/capture_color_frame.py" \
  --output "${output}" \
  --timeout-sec 45

shutdown_launch
trap - EXIT INT TERM
echo "Frame: ${output}"
echo "Launch log: ${launch_log}"
