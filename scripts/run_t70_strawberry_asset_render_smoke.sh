#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t70/strawberry_asset_render_smoke_v2}"
base_domain_id="${2:-120}"
asset_revision="${3:-v2}"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf"
models_root="${repo_root}/ros2_ws/src/strawberry_sim/models"

if [[ -e "${output_dir}" ]]; then
  echo "Output path already exists: ${output_dir}" >&2
  exit 4
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + 3 > 232)); then
  echo "Four smoke ROS domain IDs must fit in [0, 232]." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_dir}"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 100); do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

index=0
for variant in B C; do
  for maturity in RIPE UNRIPE; do
    label="${variant}_${maturity}"
    scenario_dir="${output_dir}/${label}"
    mkdir -p "${scenario_dir}"
    export ROS_DOMAIN_ID=$((base_domain_id + index))
    python3 "${repo_root}/tools/simulation/materialize_strawberry_asset_world.py" \
      --base-world "${base_world}" --models-root "${models_root}" \
      --variant "${variant}" --asset-revision "${asset_revision}" \
      --target-maturity "${maturity}" --target-position-m 0.47 -0.075 0.50 \
      --output-world "${scenario_dir}/world.sdf" \
      --output-receipt "${scenario_dir}/receipt.json"
    setsid ros2 launch strawberry_bringup system.launch.py \
      headless:=true world_file:="${scenario_dir}/world.sdf" \
      start_perception:=false start_oracle_provider:=false \
      start_manipulation:=false start_orchestrator:=false \
      enable_attachment:=false enable_pose_control:=false \
      >"${scenario_dir}/launch.log" 2>&1 &
    launch_pid=$!
    timeout --signal=TERM 45 python "${repo_root}/scripts/capture_one_rgb_frame.py" \
      --output "${scenario_dir}/rgb.png" --timeout-sec 35 \
      >"${scenario_dir}/capture.log" 2>&1
    echo "Captured ${label}."
    shutdown_launch
    index=$((index + 1))
  done
done

trap - EXIT INT TERM
echo "Captured four ${asset_revision} renderer-smoke images without perception or motion."
