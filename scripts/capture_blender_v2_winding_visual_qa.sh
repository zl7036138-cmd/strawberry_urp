#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${repo_root}/artifacts/simulation/blender_v2_body_winding_visual_qa_v1"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ ! -e "${output_dir}" ]] || {
  echo "Visual-QA output already exists: ${output_dir}" >&2
  exit 4
}
mkdir -p "${output_dir}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u

launch_pid=""
shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    kill -0 -- "-${launch_pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

index=0
for target in strawberry_1 strawberry_2; do
  target_dir="${output_dir}/${target}"
  mkdir -p "${target_dir}"
  python3 "${repo_root}/scripts/materialize_blender_v2_single_fruit.py" \
    --base-world "${base_world}" \
    --target "${target}" \
    --output-world "${target_dir}/world.sdf" \
    --output-receipt "${target_dir}/scene_receipt.json"
  export ROS_DOMAIN_ID=$((226 + index))
  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true \
    world_file:="${target_dir}/world.sdf" \
    start_perception:=false \
    start_oracle_provider:=false \
    start_manipulation:=false \
    start_orchestrator:=false \
    enable_attachment:=false \
    enable_pose_control:=false \
    >"${target_dir}/launch.log" 2>&1 &
  launch_pid=$!
  timeout --signal=TERM 60 python3 \
    "${repo_root}/scripts/capture_one_rgb_frame.py" \
    --output "${target_dir}/rgb.png" \
    --timeout-sec 45 \
    >"${target_dir}/capture.log" 2>&1
  shutdown_launch
  index=$((index + 1))
done

trap - EXIT INT TERM
sha256sum \
  "${output_dir}/strawberry_1/rgb.png" \
  "${output_dir}/strawberry_2/rgb.png" \
  >"${output_dir}/image_sha256.txt"
echo "${output_dir}"
