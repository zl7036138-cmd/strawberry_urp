#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t70/shadow_multiposition_fixed_window_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640/weights/best.pt}"
base_domain_id="${3:-195}"
manifest_path="${4:-${repo_root}/config/t70_shadow_multiposition_fixed_window.json}"
scene_config="${repo_root}/config/t70_scene_conditions_v2.json"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + 14 > 232)); then
  echo "Fifteen diagnostic ROS domain IDs must fit in [0, 232]." >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && \
  [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi

python3 "${repo_root}/scripts/validate_t70_multiposition_diagnostic.py" \
  --manifest "${manifest_path}" --model "${model_path}"
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_t70_multiposition_diagnostic.py" \
    --manifest "${manifest_path}" --model "${model_path}" --emit-tsv
)

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

scenario_index=0
for row in "${scenario_rows[@]}"; do
  IFS=$'\t' read -r scenario_id position_label condition_label lighting occlusion \
    target_model target_id x y z <<<"${row}"
  condition_dir="${output_dir}/${scenario_id}"
  world_path="${condition_dir}/world.sdf"
  receipt_path="${condition_dir}/receipt.json"
  export ROS_DOMAIN_ID=$((base_domain_id + scenario_index))
  mkdir -p "${condition_dir}"
  echo "[$((scenario_index + 1))/15] ${scenario_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${scene_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" --seed 20260710 \
    --target-position-m "${x}" "${y}" "${z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" \
    enable_pose_control:=true start_perception:=true \
    start_oracle_provider:=false start_manipulation:=false \
    start_orchestrator:=false enable_attachment:=false \
    model_path:="${model_path}" confidence_threshold:=0.31 \
    perception_detections_topic:=/strawberry/shadow/detections \
    perception_target_topic:=/strawberry/shadow/target_pose \
    >"${condition_dir}/launch.log" 2>&1 &
  launch_pid=$!

  probe_status=0
  timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
    --receipt "${receipt_path}" \
    --output-json "${condition_dir}/condition_probe.json" \
    --sensor-timeout-sec 45 \
    >"${condition_dir}/condition_probe.log" 2>&1 || probe_status=$?
  if ((probe_status == 0)); then
    window_status=0
    timeout --signal=TERM 90 ros2 run strawberry_perception shadow_window_probe \
      --output-json "${condition_dir}/shadow_window.json" \
      --scenario-id "${scenario_id}" --lighting "${lighting}" \
      --occlusion "${occlusion}" --frames 60 --timeout-sec 45 \
      --post-window-wait-sec 1.0 \
      >"${condition_dir}/shadow_window.log" 2>&1 || window_status=$?
  else
    window_status=125
    echo "condition probe failed; Shadow window skipped" \
      >"${condition_dir}/shadow_window.log"
  fi
  echo "[$((scenario_index + 1))/15] probe=${probe_status}, window=${window_status}"
  shutdown_launch
  scenario_index=$((scenario_index + 1))
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_t70_multiposition_diagnostic.py" \
  --output-dir "${output_dir}" --manifest "${manifest_path}" \
  --model "${model_path}" --base-domain-id "${base_domain_id}"
