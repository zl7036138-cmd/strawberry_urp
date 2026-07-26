#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
manifest_path="${1:-${repo_root}/config/p3_formal_matrix_v1.json}"
preflight_path="${2:-${repo_root}/artifacts/p3/p3_formal_matrix_preflight_v1/preflight_receipt.json}"
output_dir="${3:-${repo_root}/results/p3/formal_matrix_v1}"
resume="${4:-false}"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if [[ "${resume}" != "true" && "${resume}" != "false" ]]; then
  echo "resume must be true or false" >&2
  exit 2
fi

python3 "${repo_root}/scripts/validate_p3_formal_matrix.py" \
  --manifest "${manifest_path}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u

claim_arguments=(
  --manifest "${manifest_path}"
  --preflight "${preflight_path}"
  --result-dir "${output_dir}"
)
if [[ "${resume}" == "true" ]]; then
  claim_arguments+=(--resume)
fi
python3 "${repo_root}/scripts/claim_p3_formal_matrix.py" "${claim_arguments[@]}"

IFS=$'\t' read -r \
  model_path confidence_threshold scene_config base_world materialization_id \
  minimum_positive_frames minimum_negative_frames startup_timeout trial_timeout outer_timeout \
  control_position_tolerance maximum_attempts claim_path preflight_dir \
  default_result_dir < <(
    python3 "${repo_root}/scripts/validate_p3_formal_matrix.py" \
      --manifest "${manifest_path}" --emit-runtime-tsv
  )
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_p3_formal_matrix.py" \
    --manifest "${manifest_path}" --emit-schedule-tsv
)
if [[ "${#scenario_rows[@]}" -ne 165 ]]; then
  echo "formal validator did not emit 165 scenarios" >&2
  exit 2
fi

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
  IFS=$'\t' read -r \
    order_index domain_id trial_id kind lighting occlusion position_label \
    simulation_seed expected_maturity expected_outcome target_model target_id \
    target_x target_y target_z \
    park1_model park1_id park1_x park1_y park1_z \
    park2_model park2_id park2_x park2_y park2_z <<<"${row}"
  printf -v order_label '%03d' "${order_index}"
  trial_dir="${output_dir}/trials/${order_label}"
  mkdir -p "${trial_dir}"

  behavior_recorded=false
  next_attempt=0
  for attempt_index in $(seq 1 "${maximum_attempts}"); do
    printf -v attempt_label 'attempt_%02d' "${attempt_index}"
    attempt_dir="${trial_dir}/${attempt_label}"
    classification="$(python3 "${repo_root}/scripts/classify_p3_formal_attempt.py" --attempt-dir "${attempt_dir}")"
    if [[ "${classification}" == "BEHAVIOR_RECORDED" || "${classification}" == "BEHAVIOR_STATUS_UNKNOWN" ]]; then
      behavior_recorded=true
      break
    fi
    if [[ "${classification}" == "MISSING" ]]; then
      next_attempt="${attempt_index}"
      break
    fi
  done
  if [[ "${behavior_recorded}" == "true" ]]; then
    echo "[${order_index}/165] preserved behavioral result; skipping ${trial_id}"
    continue
  fi
  if ((next_attempt == 0)); then
    echo "[${order_index}/165] infrastructure attempts exhausted: ${trial_id}" >&2
    continue
  fi

  printf -v attempt_label 'attempt_%02d' "${next_attempt}"
  attempt_dir="${trial_dir}/${attempt_label}"
  mkdir -p "${attempt_dir}"
  world_path="${attempt_dir}/world.sdf"
  receipt_path="${attempt_dir}/condition_receipt.json"
  probe_path="${attempt_dir}/condition_probe.json"
  launch_log="${attempt_dir}/launch.log"
  result_path="${attempt_dir}/result.json"
  client_log="${attempt_dir}/client.log"
  probe_log="${attempt_dir}/condition_probe.log"
  export ROS_DOMAIN_ID="${domain_id}"
  echo "[${order_index}/165] ${trial_id}, attempt=${next_attempt}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, gz_seed=${simulation_seed}"

  python3 "${repo_root}/scripts/materialize_scene_condition.py" \
    --base-world "${base_world}" --config "${scene_config}" \
    --lighting "${lighting}" --occlusion "${occlusion}" \
    --seed "${materialization_id}" \
    --target-position-m "${target_x}" "${target_y}" "${target_z}" \
    --output-world "${world_path}" --output-receipt "${receipt_path}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true world_file:="${world_path}" simulation_seed:="${simulation_seed}" \
    enable_attachment:=true enable_pose_control:=true \
    start_perception:=true start_oracle_provider:=false \
    start_manipulation:=true start_orchestrator:=true \
    model_path:="${model_path}" confidence_threshold:="${confidence_threshold}" \
    perception_detections_topic:=/strawberry/detections \
    perception_target_topic:=/strawberry/target_pose \
    target_source:=perception control_target_topic:=/strawberry/target_pose \
    shadow_enabled:=false >"${launch_log}" 2>&1 &
  launch_pid=$!

  probe_status=0
  timeout --signal=TERM 120 ros2 run strawberry_sim scene_condition_probe \
    --receipt "${receipt_path}" --output-json "${probe_path}" \
    --sensor-timeout-sec 45 >"${probe_log}" 2>&1 || probe_status=$?
  if ((probe_status != 0)); then
    echo "[${order_index}/165] condition probe failed with ${probe_status}"
    shutdown_launch
    continue
  fi

  client_arguments=(
    --formal-kind "${kind}"
    --formal-lighting "${lighting}"
    --formal-occlusion "${occlusion}"
    --formal-position "${position_label}"
    --formal-simulation-seed "${simulation_seed}"
    --formal-order-index "${order_index}"
    --condition-receipt "${receipt_path}"
    --condition-probe "${probe_path}"
    --startup-timeout-sec "${startup_timeout}"
    --trial-timeout-sec "${trial_timeout}"
    --control-topic /strawberry/target_pose
    --detections-topic /strawberry/detections
    --target-source perception
    --scenario-id "${trial_id}"
    --expected-target-id "${target_id}"
    --target-model-name "${target_model}"
    --target-position-m "${target_x}" "${target_y}" "${target_z}"
    --control-position-tolerance-m "${control_position_tolerance}"
    --park-model-pose "${park1_model}" "${park1_id}" "${park1_x}" "${park1_y}" "${park1_z}"
    --park-model-pose "${park2_model}" "${park2_id}" "${park2_x}" "${park2_y}" "${park2_z}"
    --output "${result_path}"
  )
  if [[ "${kind}" == "NEGATIVE" ]]; then
    client_arguments+=(
      --expect-no-pick
      --minimum-detection-frames-after-scene "${minimum_negative_frames}"
    )
  else
    client_arguments+=(
      --permit-missing-positive-target
      --minimum-detection-frames-after-scene "${minimum_positive_frames}"
    )
  fi
  client_status=0
  timeout --signal=TERM "${outer_timeout}" \
    python "${repo_root}/scripts/test_formal_p3_trial.py" \
    "${client_arguments[@]}" >"${client_log}" 2>&1 || client_status=$?
  echo "[${order_index}/165] client status ${client_status}"
  shutdown_launch
done

trap - EXIT INT TERM
python "${repo_root}/scripts/summarize_p3_formal_matrix.py" \
  --output-dir "${output_dir}" --manifest "${manifest_path}" \
  --preflight "${preflight_path}" --model "${model_path}"
