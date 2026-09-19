#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_root="${1:-${repo_root}/data/processed/blender_v2_adaptation_capture_v1}"
base_domain_id="${2:-210}"
manifest_path="${3:-${repo_root}/config/blender_v2_adaptation_capture_v1.json}"
condition_config="${repo_root}/config/blender_v2_adaptation_scene_conditions.json"
scene_manifest="${repo_root}/ros2_ws/src/strawberry_sim/config/scene.yaml"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ "${base_domain_id}" =~ ^[0-9]+$ ]] && ((base_domain_id + 11 <= 232)) || {
  echo "Twelve ROS domain IDs must fit in [0,232]." >&2
  exit 2
}
if [[ -e "${output_root}" ]] && \
  [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_root}" >&2
  exit 4
fi

python3 "${repo_root}/scripts/validate_blender_v2_adaptation_capture.py" \
  --manifest "${manifest_path}"
mapfile -t group_rows < <(
  python3 "${repo_root}/scripts/validate_blender_v2_adaptation_capture.py" \
    --manifest "${manifest_path}" --emit-groups-tsv
)
if ((${#group_rows[@]} != 12)); then
  echo "Capture manifest did not emit exactly 12 groups." >&2
  exit 5
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_root}/groups" "${output_root}/logs" \
  "${output_root}/worlds"

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

group_index=0
for row in "${group_rows[@]}"; do
  IFS=$'\t' read -r group_id split maturity condition_id lighting occlusion \
    materialization_id x y z <<<"${row}"
  export ROS_DOMAIN_ID=$((base_domain_id + group_index))
  world_dir="${output_root}/worlds/${group_id}"
  world_path="${world_dir}/world.sdf"
  receipt_path="${world_dir}/receipt.json"
  launch_log="${output_root}/logs/${group_id}.launch.log"
  capture_log="${output_root}/logs/${group_id}.capture.log"
  mkdir -p "${world_dir}"
  echo "[$((group_index + 1))/12] ${group_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  domain_nodes="$({
    timeout 5 ros2 node list --no-daemon --spin-time 1.0 --all || true
  } | grep -Ev '^/?_ros2cli_[0-9]+$' || true)"
  if [[ -n "${domain_nodes}" ]]; then
    echo "ROS domain ${ROS_DOMAIN_ID} is not empty:" >&2
    echo "${domain_nodes}" >&2
    exit 6
  fi

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${condition_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" \
    --seed "${materialization_id}" --target-position-m "${x}" "${y}" "${z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" \
    scene_config_file:="${scene_manifest}" enable_pose_control:=true \
    start_perception:=false start_oracle_provider:=false \
    start_manipulation:=false start_orchestrator:=false \
    enable_attachment:=false \
    >"${launch_log}" 2>&1 &
  launch_pid=$!

  capture_status=0
  timeout --signal=TERM 180 ros2 run strawberry_sim synthetic_capture \
    --manifest "${manifest_path}" --scene-config "${condition_config}" \
    --receipt "${receipt_path}" --output-root "${output_root}" \
    --group-id "${group_id}" --split "${split}" --maturity "${maturity}" \
    --condition-id "${condition_id}" --sensor-timeout-sec 45 \
    >"${capture_log}" 2>&1 || capture_status=$?
  shutdown_launch
  if ((capture_status != 0)); then
    echo "Synthetic capture failed for ${group_id}: ${capture_status}" >&2
    exit "${capture_status}"
  fi
  if grep -Eqi \
    "Traceback|exception was never retrieved|process has died|RuntimeError" \
    "${launch_log}" "${capture_log}"; then
    echo "Capture logs contain a runtime failure for ${group_id}." >&2
    exit 7
  fi
  group_index=$((group_index + 1))
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_blender_v2_adaptation_capture.py" \
  --output-root "${output_root}" --manifest "${manifest_path}"
