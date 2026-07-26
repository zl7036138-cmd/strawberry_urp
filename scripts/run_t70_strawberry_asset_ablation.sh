#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t70/strawberry_asset_ablation_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640/weights/best.pt}"
base_domain_id="${3:-100}"
manifest_path="${4:-${repo_root}/config/t70_strawberry_asset_ablation_v1.json}"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf"
models_root="${repo_root}/ros2_ws/src/strawberry_sim/models"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + 17 > 232)); then
  echo "Eighteen diagnostic ROS domain IDs must fit in [0, 232]." >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi

python3 "${repo_root}/scripts/validate_t70_strawberry_asset_ablation.py" \
  --manifest "${manifest_path}" --model "${model_path}"
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_t70_strawberry_asset_ablation.py" \
    --manifest "${manifest_path}" --model "${model_path}" --emit-tsv
)
asset_revision="$(
  python3 "${repo_root}/scripts/validate_t70_strawberry_asset_ablation.py" \
    --manifest "${manifest_path}" --model "${model_path}" --emit-asset-revision
)"

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
  IFS=$'\t' read -r scenario_id variant maturity position_label x y z <<<"${row}"
  scenario_dir="${output_dir}/${scenario_id}"
  export ROS_DOMAIN_ID=$((base_domain_id + scenario_index))
  mkdir -p "${scenario_dir}"
  echo "[$((scenario_index + 1))/18] ${scenario_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  python3 "${repo_root}/tools/simulation/materialize_strawberry_asset_world.py" \
    --base-world "${base_world}" --models-root "${models_root}" \
    --variant "${variant}" --asset-revision "${asset_revision}" \
    --target-maturity "${maturity}" \
    --target-position-m "${x}" "${y}" "${z}" \
    --output-world "${scenario_dir}/world.sdf" \
    --output-receipt "${scenario_dir}/receipt.json" \
    >"${scenario_dir}/materialize.log" 2>&1

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${scenario_dir}/world.sdf" \
    start_perception:=true start_oracle_provider:=false \
    start_manipulation:=false start_orchestrator:=false \
    enable_attachment:=false enable_pose_control:=false \
    model_path:="${model_path}" confidence_threshold:=0.31 \
    perception_detections_topic:=/strawberry/shadow/detections \
    perception_target_topic:=/strawberry/shadow/target_pose \
    >"${scenario_dir}/launch.log" 2>&1 &
  launch_pid=$!

  window_status=0
  timeout --signal=TERM 120 ros2 run strawberry_perception shadow_window_probe \
    --output-json "${scenario_dir}/shadow_window.json" \
    --scenario-id "${scenario_id}" --lighting nominal --occlusion none \
    --warmup-frames 30 --frames 60 --timeout-sec 75 \
    --post-window-wait-sec 1.0 \
    --window-boundary after_fixed_warmup_before_robot_motion \
    >"${scenario_dir}/shadow_window.log" 2>&1 || window_status=$?

  capture_status=0
  if [[ "${position_label}" == "center" && "${window_status}" -eq 0 ]]; then
    timeout --signal=TERM 30 python "${repo_root}/scripts/capture_one_rgb_frame.py" \
      --output "${scenario_dir}/rgb.png" --timeout-sec 20 \
      >"${scenario_dir}/rgb_capture.log" 2>&1 || capture_status=$?
  fi
  echo "[$((scenario_index + 1))/18] window=${window_status}, capture=${capture_status}"
  shutdown_launch
  scenario_index=$((scenario_index + 1))
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_t70_strawberry_asset_ablation.py" \
  --output-dir "${output_dir}" --manifest "${manifest_path}" --model "${model_path}"
