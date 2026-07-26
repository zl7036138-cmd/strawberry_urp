#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/dual_camera_smoke_v1}"
domain_id="${2:-229}"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ ! -e "${output_dir}" ]] || {
  echo "Output already exists; choose a fresh directory: ${output_dir}" >&2
  exit 4
}
[[ "${domain_id}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]] || {
  echo "ROS domain ID must be in [0,232]" >&2
  exit 2
}

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"
mkdir -p "${output_dir}"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 200); do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_sim sim.launch.py \
  headless:=true camera_mount:=dual enable_attachment:=false \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

timeout --signal=TERM 100 ros2 run strawberry_sim runtime_health_check \
  --duration-sec 6 --startup-timeout-sec 70 --report-interval-sec 3 \
  --camera-prefix /camera/base \
  --camera-frame strawberry_base_camera_optical_frame \
  --expected-width 320 --expected-height 240 \
  --output "${output_dir}/base_runtime_health.json" \
  >"${output_dir}/base_runtime_health.log" 2>&1

timeout --signal=TERM 45 ros2 run strawberry_sim runtime_health_check \
  --duration-sec 6 --startup-timeout-sec 30 --report-interval-sec 3 \
  --camera-prefix /camera/wrist \
  --camera-frame strawberry_wrist_camera_optical_frame \
  --output "${output_dir}/wrist_runtime_health.json" \
  >"${output_dir}/wrist_runtime_health.log" 2>&1

python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/base/color/image_raw \
  --output "${output_dir}/base_ready_rgb.png" --timeout-sec 30 \
  >"${output_dir}/base_capture.log" 2>&1
python "${repo_root}/scripts/capture_one_rgb_frame.py" \
  --topic /camera/wrist/color/image_raw \
  --output "${output_dir}/wrist_ready_rgb.png" --timeout-sec 30 \
  >"${output_dir}/wrist_capture.log" 2>&1

python "${repo_root}/scripts/capture_tf_transform.py" \
  --source-frame panda_link0 \
  --target-frame strawberry_base_camera_optical_frame \
  --output "${output_dir}/base_camera_tf.json" \
  >"${output_dir}/base_camera_tf.log" 2>&1
python "${repo_root}/scripts/capture_tf_transform.py" \
  --source-frame panda_link0 \
  --target-frame strawberry_wrist_camera_optical_frame \
  --output "${output_dir}/wrist_camera_tf.json" \
  >"${output_dir}/wrist_camera_tf.log" 2>&1

shutdown_launch
trap - EXIT INT TERM
if grep -Eqi "Traceback|process has died|RuntimeError" \
  "${output_dir}/launch.log" \
  "${output_dir}/base_runtime_health.log" \
  "${output_dir}/wrist_runtime_health.log"; then
  echo "Dual-camera smoke logs contain an unhandled failure" >&2
  exit 1
fi
echo "Dual camera smoke: ${output_dir}"
