#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
manifest_path="${1:-${repo_root}/config/p4_sim_adapt_qualification_v1.json}"
preflight_path="${2:-${repo_root}/artifacts/p4/p4_sim_adapt_qualification_preflight_v1.json}"
output_dir="${3:-${repo_root}/results/p4/sim_adapt_qualification_v1}"
resume="${4:-false}"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if [[ "${resume}" != "true" && "${resume}" != "false" ]]; then
  echo "resume must be true or false" >&2
  exit 2
fi

python3 "${repo_root}/scripts/validate_p4_sim_adapt_qualification.py" --manifest "${manifest_path}"
claim_args=(--manifest "${manifest_path}" --preflight "${preflight_path}" --result-dir "${output_dir}")
if [[ "${resume}" == "true" ]]; then claim_args+=(--resume); fi
python3 "${repo_root}/scripts/claim_p4_sim_adapt_qualification.py" "${claim_args[@]}"

IFS=$'\t' read -r model_path confidence_threshold scene_config base_world \
  materialization_id simulation_seed fixed_frames post_wait claim_path < <(
    python3 "${repo_root}/scripts/validate_p4_sim_adapt_qualification.py" \
      --manifest "${manifest_path}" --emit-runtime-tsv
  )
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_p4_sim_adapt_qualification.py" \
    --manifest "${manifest_path}" --emit-schedule-tsv
)
if [[ "${#scenario_rows[@]}" -ne 30 ]]; then
  echo "qualification validator did not emit 30 scenarios" >&2
  exit 2
fi

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

for row in "${scenario_rows[@]}"; do
  IFS=$'\t' read -r order_index domain_id scenario_id maturity position_label \
    condition_label lighting occlusion target_model target_id target_x target_y target_z \
    park1_model park1_id park1_x park1_y park1_z \
    park2_model park2_id park2_x park2_y park2_z <<<"${row}"
  printf -v order_label '%03d' "${order_index}"
  trial_dir="${output_dir}/trials/${order_label}"
  if [[ -f "${trial_dir}/shadow_window.json" && -f "${trial_dir}/shutdown.ok" ]]; then
    echo "[${order_index}/30] preserving completed qualification window ${scenario_id}"
    continue
  fi
  if [[ -e "${trial_dir}" ]] && [[ -n "$(find "${trial_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "[${order_index}/30] incomplete prior scenario cannot be overwritten: ${scenario_id}" >&2
    continue
  fi
  mkdir -p "${trial_dir}"
  world_path="${trial_dir}/world.sdf"
  receipt_path="${trial_dir}/condition_receipt.json"
  export ROS_DOMAIN_ID="${domain_id}"
  echo "[${order_index}/30] ${scenario_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${scene_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" \
    --seed "${materialization_id}" \
    --target-position-m "${target_x}" "${target_y}" "${target_z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" simulation_seed:="${simulation_seed}" \
    enable_pose_control:=true start_perception:=true \
    start_oracle_provider:=false start_manipulation:=false \
    start_orchestrator:=false enable_attachment:=false \
    model_path:="${model_path}" confidence_threshold:="${confidence_threshold}" \
    perception_detections_topic:=/strawberry/shadow/detections \
    perception_target_topic:=/strawberry/shadow/target_pose \
    >"${trial_dir}/launch.log" 2>&1 &
  launch_pid=$!

  probe_status=0
  timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
    --receipt "${receipt_path}" --output-json "${trial_dir}/condition_probe.json" \
    --sensor-timeout-sec 45 >"${trial_dir}/condition_probe.log" 2>&1 || probe_status=$?
  configure_status=125
  window_status=125
  if ((probe_status == 0)); then
    configure_status=0
    timeout --signal=TERM 90 python "${repo_root}/scripts/configure_p4_qualification_scene.py" \
      --target-id "${target_id}" --maturity "${maturity}" \
      --model-pose "${target_model}" "${target_id}" "${target_x}" "${target_y}" "${target_z}" \
      --model-pose "${park1_model}" "${park1_id}" "${park1_x}" "${park1_y}" "${park1_z}" \
      --model-pose "${park2_model}" "${park2_id}" "${park2_x}" "${park2_y}" "${park2_z}" \
      --settled-camera-frames 10 --output "${trial_dir}/scene_configuration.json" \
      >"${trial_dir}/scene_configuration.log" 2>&1 || configure_status=$?
  fi
  if ((configure_status == 0)); then
    window_status=0
    timeout --signal=TERM 90 ros2 run strawberry_perception shadow_window_probe \
      --output-json "${trial_dir}/shadow_window.json" \
      --scenario-id "${scenario_id}" --lighting "${lighting}" \
      --occlusion "${occlusion}" --frames "${fixed_frames}" --timeout-sec 45 \
      --post-window-wait-sec "${post_wait}" \
      --window-boundary after_p4_scene_configuration_before_robot_motion \
      >"${trial_dir}/shadow_window.log" 2>&1 || window_status=$?
  fi
  echo "[${order_index}/30] condition=${probe_status}, configure=${configure_status}, window=${window_status}"
  shutdown_launch
  touch "${trial_dir}/shutdown.ok"
done

trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_p4_sim_adapt_qualification.py" \
  --manifest "${manifest_path}" --output-dir "${output_dir}"
