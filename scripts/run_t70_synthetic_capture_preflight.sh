#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runner_path="$(realpath "${BASH_SOURCE[0]}")"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_root="${1:-${repo_root}/data/processed/t70_sim_adaptation_preflight_v1}"
base_domain_id="${2:-140}"
manifest_path="${3:-${repo_root}/config/t70_synthetic_capture_preflight.json}"
scene_config="${repo_root}/config/t70_synthetic_capture_scene_conditions.json"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + 35 > 232)); then
  echo "Thirty-six synthetic capture ROS domain IDs must fit in [0, 232]." >&2
  exit 2
fi
if [[ -e "${output_root}" ]] && \
  [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_root}" >&2
  exit 4
fi

python3 "${repo_root}/scripts/validate_t70_synthetic_capture_preflight.py" \
  --manifest "${manifest_path}"
mapfile -t group_rows < <(
  python3 "${repo_root}/scripts/validate_t70_synthetic_capture_preflight.py" \
    --manifest "${manifest_path}" --emit-groups-tsv
)
if ((${#group_rows[@]} != 36)); then
  echo "Capture manifest did not emit exactly 36 groups." >&2
  exit 5
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_root}/groups" "${output_root}/logs" "${output_root}/worlds"

launch_pid=""
shutdown_launch() {
  if [[ -z "${launch_pid}" ]] || ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
    launch_pid=""
    return
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM

group_index=0
for row in "${group_rows[@]}"; do
  IFS=$'\t' read -r group_id split maturity class_id target_id target_model \
    condition_id lighting occlusion materialization_id x y z <<<"${row}"
  export ROS_DOMAIN_ID=$((base_domain_id + group_index))
  world_dir="${output_root}/worlds/${group_id}"
  world_path="${world_dir}/world.sdf"
  receipt_path="${world_dir}/receipt.json"
  launch_log="${output_root}/logs/${group_id}.launch.log"
  capture_log="${output_root}/logs/${group_id}.capture.log"
  mkdir -p "${world_dir}"
  echo "[$((group_index + 1))/36] ${group_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  domain_nodes="$({ timeout 5 ros2 node list --no-daemon --spin-time 1.0 --all || true; } | grep -Ev '^/?_ros2cli_[0-9]+$' || true)"
  if [[ -n "${domain_nodes}" ]]; then
    echo "ROS domain ${ROS_DOMAIN_ID} is not empty:" >&2
    echo "${domain_nodes}" >&2
    exit 6
  fi

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${scene_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" \
    --seed "${materialization_id}" --target-position-m "${x}" "${y}" "${z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" enable_pose_control:=true \
    start_perception:=false start_oracle_provider:=false \
    start_manipulation:=false start_orchestrator:=false \
    enable_attachment:=false \
    >"${launch_log}" 2>&1 &
  launch_pid=$!

  capture_status=0
  timeout --signal=TERM 180 ros2 run strawberry_sim synthetic_capture \
    --manifest "${manifest_path}" --scene-config "${scene_config}" \
    --receipt "${receipt_path}" --output-root "${output_root}" \
    --group-id "${group_id}" --split "${split}" --maturity "${maturity}" \
    --condition-id "${condition_id}" --sensor-timeout-sec 45 \
    >"${capture_log}" 2>&1 || capture_status=$?
  shutdown_launch
  if ((capture_status != 0)); then
    echo "Synthetic capture failed for ${group_id} with status ${capture_status}." >&2
    exit "${capture_status}"
  fi
  if grep -Eqi "Traceback|exception was never retrieved|process has died" \
    "${launch_log}" "${capture_log}"; then
    echo "Synthetic capture logs contain a runtime failure for ${group_id}." >&2
    exit 7
  fi
  group_index=$((group_index + 1))
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_t70_synthetic_capture_preflight.py" \
  --output-root "${output_root}" --manifest "${manifest_path}" \
  --runner "${runner_path}" --base-domain-id "${base_domain_id}"
