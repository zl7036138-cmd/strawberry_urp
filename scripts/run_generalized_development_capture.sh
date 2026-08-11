#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_root="${1:-${repo_root}/data/processed/generalized_development_capture_v1}"
ros_domain_id="${2:-214}"
plan_path="${3:-${repo_root}/config/generalized_development_capture_v1.json}"
formal_matrix="${repo_root}/config/generalized_harvest_matrix_v1.json"
base_scene="${repo_root}/ros2_ws/src/strawberry_sim/config/scene.yaml"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ "${ros_domain_id}" =~ ^[0-9]+$ ]] && ((ros_domain_id <= 232)) || {
  echo "ROS domain ID must be in [0,232]." >&2
  exit 2
}
if [[ -e "${output_root}" ]] && \
  [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_root}" >&2
  exit 4
fi

mapfile -t scene_rows < <(
  python3 "${repo_root}/scripts/prepare_generalized_development_capture.py" \
    --plan "${plan_path}" --formal-matrix "${formal_matrix}" --emit-scenes-tsv
)
if ((${#scene_rows[@]} != 120)); then
  echo "Development capture plan did not emit exactly 120 scenes." >&2
  exit 5
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ros_domain_id}"
mkdir -p "${output_root}/worlds" "${output_root}/logs"

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

scene_index=0
for row in "${scene_rows[@]}"; do
  IFS=$'\t' read -r sample_id split seed profile plant_count position_band occlusion <<<"${row}"
  scene_dir="${output_root}/worlds/${sample_id}"
  scene_path="${scene_dir}/generalized_seed_$(printf '%06d' "${seed}").yaml"
  world_path="${scene_dir}/generalized_seed_$(printf '%06d' "${seed}").sdf"
  receipt_path="${scene_dir}/generalized_seed_$(printf '%06d' "${seed}").receipt.json"
  launch_log="${output_root}/logs/${sample_id}.launch.log"
  capture_log="${output_root}/logs/${sample_id}.capture.log"
  echo "[$((scene_index + 1))/120] ${sample_id}"

  ros2 run strawberry_sim generate_generalized_scene \
    --base-scene "${base_scene}" --base-world "${base_world}" \
    --output-dir "${scene_dir}" --seed "${seed}" --profile "${profile}" \
    --plant-count "${plant_count}" --position-band "${position_band}" \
    --occlusion "${occlusion}" >"${output_root}/logs/${sample_id}.generate.log"

  setsid ros2 launch strawberry_sim sim.launch.py \
    headless:=true world_file:="${world_path}" scene_config_file:="${scene_path}" \
    simulation_seed:="${seed}" camera_mount:=fixed enable_attachment:=false \
    enable_pose_control:=false >"${launch_log}" 2>&1 &
  launch_pid=$!

  capture_status=0
  timeout --signal=TERM 120 ros2 run strawberry_sim generalized_development_capture \
    --plan "${plan_path}" --formal-matrix "${formal_matrix}" \
    --scene-config "${scene_path}" --scene-receipt "${receipt_path}" \
    --output-root "${output_root}" --split "${split}" --seed "${seed}" \
    --sensor-timeout-sec 45 >"${capture_log}" 2>&1 || capture_status=$?
  shutdown_launch
  if ((capture_status != 0)); then
    echo "Development capture failed for ${sample_id}: ${capture_status}" >&2
    exit "${capture_status}"
  fi
  if grep -Eqi "Traceback|process has died|RuntimeError" "${launch_log}" "${capture_log}"; then
    echo "Capture logs contain a runtime failure for ${sample_id}." >&2
    exit 7
  fi
  scene_index=$((scene_index + 1))
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_generalized_development_capture.py" \
  --plan "${plan_path}" --formal-matrix "${formal_matrix}" \
  --output-root "${output_root}"
