#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t70/scene_condition_injection_pre_gate_v1}"
base_domain_id="${2:-180}"
config_path="${3:-${repo_root}/config/t70_scene_conditions.json}"
base_world="${4:-${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf}"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if [[ ! "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + 8 > 232)); then
  echo "Nine ROS domain IDs starting at base_domain_id must fit in [0, 232]." >&2
  exit 2
fi
if [[ ! -f "${config_path}" ]] || [[ ! -f "${base_world}" ]]; then
  echo "Frozen config or base world is missing." >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && \
  [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi

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
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

condition_index=0
for lighting in dim nominal bright; do
  for occlusion in none partial heavy; do
    condition_id="${lighting}__${occlusion}"
    condition_dir="${output_dir}/${condition_id}"
    world_path="${condition_dir}/world.sdf"
    receipt_path="${condition_dir}/receipt.json"
    probe_path="${condition_dir}/probe.json"
    launch_log="${condition_dir}/launch.log"
    probe_log="${condition_dir}/probe.log"
    export ROS_DOMAIN_ID=$((base_domain_id + condition_index))
    mkdir -p "${condition_dir}"
    echo "[$((condition_index + 1))/9] ${condition_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

    python3 "${repo_root}/scripts/materialize_scene_condition.py" \
      --base-world "${base_world}" \
      --config "${config_path}" \
      --lighting "${lighting}" \
      --occlusion "${occlusion}" \
      --seed 20260710 \
      --output-world "${world_path}" \
      --output-receipt "${receipt_path}"

    domain_nodes="$({ timeout 5 ros2 node list --no-daemon \
      --spin-time 1.0 --all || true; } | grep -Ev '^/?_ros2cli_[0-9]+$' || true)"
    if [[ -n "${domain_nodes}" ]]; then
      printf 'ROS domain %s is not empty:\n%s\n' \
        "${ROS_DOMAIN_ID}" "${domain_nodes}" >"${probe_log}"
      condition_index=$((condition_index + 1))
      continue
    fi

    setsid ros2 launch strawberry_bringup system.launch.py \
      headless:=true \
      world_file:="${world_path}" \
      enable_pose_control:=true \
      >"${launch_log}" 2>&1 &
    launch_pid=$!

    probe_status=0
    timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
      --receipt "${receipt_path}" \
      --output-json "${probe_path}" \
      --sensor-timeout-sec 45 \
      >"${probe_log}" 2>&1 || probe_status=$?
    echo "[$((condition_index + 1))/9] probe status ${probe_status}"
    shutdown_launch
    condition_index=$((condition_index + 1))
  done
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_scene_condition_gate.py" \
  --output-dir "${output_dir}" \
  --config "${config_path}"
