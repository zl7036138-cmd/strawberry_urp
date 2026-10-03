#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
manifest_path="${1:-${repo_root}/config/rendered_occlusion_localization_matrix_v1.json}"
output_dir="${2:-${repo_root}/results/development/rendered_occlusion_localization_matrix_v1}"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
[[ ! -e "${output_dir}" ]] || {
  echo "Output already exists; choose a fresh directory: ${output_dir}" >&2
  exit 4
}

python3 "${repo_root}/scripts/validate_rendered_occlusion_localization_matrix.py" \
  --manifest "${manifest_path}"
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_rendered_occlusion_localization_matrix.py" \
    --manifest "${manifest_path}" --emit-tsv
)
mapfile -t runtime_values < <(
  python3 - "${manifest_path}" <<'PY'
import json,sys
d=json.load(open(sys.argv[1], encoding="utf-8"))["runtime"]
for key in (
    "samples_per_scenario",
    "simulation_seed",
    "base_ros_domain_id",
    "pair_wait_sec",
    "oracle_bbox_padding",
):
    print(d[key])
PY
)
samples_per_scenario="${runtime_values[0]}"
simulation_seed="${runtime_values[1]}"
base_domain_id="${runtime_values[2]}"
pair_wait_sec="${runtime_values[3]}"
oracle_bbox_padding="${runtime_values[4]}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u

simulation_share="$(ros2 pkg prefix strawberry_sim)/share/strawberry_sim"
localization_share="$(ros2 pkg prefix strawberry_localization)/share/strawberry_localization"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"
scene_manifest="${repo_root}/ros2_ws/src/strawberry_sim/config/scene.yaml"
condition_config="${repo_root}/config/rendered_occlusion_localization_conditions_v1.json"
center_config="${localization_share}/config/localization_rendered_center_v1.yaml"
geometry_config="${localization_share}/config/localization_rendered_geometry_layer_v1.yaml"
source_center_config="${repo_root}/ros2_ws/src/strawberry_localization/config/localization_rendered_center_v1.yaml"
source_geometry_config="${repo_root}/ros2_ws/src/strawberry_localization/config/localization_rendered_geometry_layer_v1.yaml"
source_localization_core="${repo_root}/ros2_ws/src/strawberry_localization/strawberry_localization/core.py"
runtime_localization_core="$(python3 - <<'PY'
from pathlib import Path
import strawberry_localization.core as core
print(Path(core.__file__).resolve())
PY
)"
cmp -s "${center_config}" "${source_center_config}" || {
  echo "Installed center-localization config is stale" >&2
  exit 5
}
cmp -s "${geometry_config}" "${source_geometry_config}" || {
  echo "Installed geometry-localization config is stale" >&2
  exit 5
}
cmp -s "${runtime_localization_core}" "${source_localization_core}" || {
  echo "Installed localization core is stale" >&2
  exit 5
}
detections_topic="/strawberry/rendered/detections"
center_topic="/strawberry/rendered/center_target_pose"
geometry_topic="/strawberry/rendered/geometry_target_pose"

mkdir -p "${output_dir}"
launch_pid=""
pipeline_pids=()

stop_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 -- "-${pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 150); do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

shutdown_all() {
  local pid
  for pid in "${pipeline_pids[@]}"; do
    stop_group "${pid}"
  done
  pipeline_pids=()
  stop_group "${launch_pid}"
  launch_pid=""
}
trap shutdown_all EXIT INT TERM

scenario_failures=0
for row in "${scenario_rows[@]}"; do
  IFS=$'\t' read -r order_index scenario_id position_label condition_label \
    lighting occlusion x y z <<<"${row}"
  scenario_dir="${output_dir}/${scenario_id}"
  world_path="${scenario_dir}/world.sdf"
  receipt_path="${scenario_dir}/condition_receipt.json"
  probe_path="${scenario_dir}/condition_probe.json"
  export ROS_DOMAIN_ID=$((base_domain_id + order_index - 1))
  mkdir -p "${scenario_dir}"
  echo "[${order_index}/15] ${scenario_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${condition_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" \
    --seed "${simulation_seed}" --target-position-m "${x}" "${y}" "${z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}" \
    >"${scenario_dir}/materialization.log" 2>&1

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" \
    scene_config_file:="${scene_manifest}" simulation_seed:="${simulation_seed}" \
    camera_mount:=fixed start_perception:=false start_oracle_provider:=false \
    start_manipulation:=false start_orchestrator:=false \
    enable_attachment:=false enable_pose_control:=true \
    >"${scenario_dir}/launch.log" 2>&1 &
  launch_pid=$!

  condition_status=0
  timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
    --receipt "${receipt_path}" --output-json "${probe_path}" \
    --sensor-timeout-sec 60 \
    >"${scenario_dir}/condition_probe.log" 2>&1 || condition_status=$?

  trial_status=125
  if ((condition_status == 0)); then
    setsid ros2 run strawberry_localization localization_node --ros-args \
      --params-file "${center_config}" \
      -r __node:=rendered_center_localization \
      -p detections_topic:="${detections_topic}" \
      -p target_pose_topic:="${center_topic}" \
      -p ground_truth_association_enabled:=false \
      -p sensor_qos_depth:=30 \
      >"${scenario_dir}/center_localization.log" 2>&1 &
    pipeline_pids+=("$!")

    setsid ros2 run strawberry_localization localization_node --ros-args \
      --params-file "${geometry_config}" \
      -r __node:=rendered_geometry_localization \
      -p detections_topic:="${detections_topic}" \
      -p target_pose_topic:="${geometry_topic}" \
      -p ground_truth_association_enabled:=false \
      -p sensor_qos_depth:=30 \
      >"${scenario_dir}/geometry_localization.log" 2>&1 &
    pipeline_pids+=("$!")

    trial_status=0
    timeout --signal=TERM 150 python3 \
      "${repo_root}/scripts/probe_paired_rendered_localization.py" \
      --output "${scenario_dir}/paired_trial.json" \
      --condition-receipt "${receipt_path}" \
      --condition-probe "${probe_path}" \
      --scenario-id "${scenario_id}" --position-label "${position_label}" \
      --occlusion "${occlusion}" --samples "${samples_per_scenario}" \
      --pair-wait-sec "${pair_wait_sec}" \
      --bbox-padding "${oracle_bbox_padding}" --timeout-sec 120 \
      --detections-topic "${detections_topic}" \
      --center-topic "${center_topic}" --geometry-topic "${geometry_topic}" \
      >"${scenario_dir}/paired_trial.log" 2>&1 || trial_status=$?
  fi

  echo "[${order_index}/15] condition=${condition_status}, trial=${trial_status}"
  if ((condition_status != 0 || trial_status != 0)); then
    scenario_failures=$((scenario_failures + 1))
  fi
  shutdown_all
  touch "${scenario_dir}/shutdown.ok"
done

trap - EXIT INT TERM
summary_status=0
python3 "${repo_root}/scripts/summarize_rendered_occlusion_localization_matrix.py" \
  --output-dir "${output_dir}" --manifest "${manifest_path}" \
  >"${output_dir}/summary.log" 2>&1 || summary_status=$?

if ((scenario_failures > 0)); then
  echo "${scenario_failures} rendered-localization scenarios failed infrastructure" >&2
fi
echo "Rendered-occlusion localization matrix: ${output_dir}/summary.json"
exit "${summary_status}"
