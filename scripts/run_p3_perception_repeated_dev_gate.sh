#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/p3/perception_repeated_dev_gate_v1}"
requested_base_domain_id="${2:-}"
manifest_path="${3:-${repo_root}/config/p3_perception_repeated_dev_gate_v1.json}"
model_override="${4:-}"

if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi

validator_common=(--manifest "${manifest_path}")
if [[ -n "${model_override}" ]]; then
  validator_common+=(--model "${model_override}")
fi
IFS=$'\t' read -r \
  model_path confidence_threshold detections_topic target_topic control_topic \
  positive_trials negative_trials minimum_detection_frames manifest_base_domain_id < <(
    python3 "${repo_root}/scripts/validate_p3_perception_repeated_dev_gate.py" \
      "${validator_common[@]}" --emit-runtime-tsv
  )
base_domain_id="${requested_base_domain_id:-${manifest_base_domain_id}}"
total_trials=$((positive_trials + negative_trials))
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  echo "base_domain_id must be a non-negative integer" >&2
  exit 2
fi
if ((base_domain_id + total_trials - 1 > 232)); then
  echo "repeated gate ROS domain range would exceed 232" >&2
  exit 2
fi

IFS=$'\t' read -r \
  positive_scenario positive_target_model positive_target_id \
  positive_x positive_y positive_z < <(
    python3 "${repo_root}/scripts/validate_p3_perception_repeated_dev_gate.py" \
      "${validator_common[@]}" --emit-positive-target-tsv
  )
IFS=$'\t' read -r \
  negative_scenario negative_target_model negative_target_id \
  negative_x negative_y negative_z < <(
    python3 "${repo_root}/scripts/validate_p3_perception_repeated_dev_gate.py" \
      "${validator_common[@]}" --emit-negative-target-tsv
  )
mapfile -t positive_parked_rows < <(
  python3 "${repo_root}/scripts/validate_p3_perception_repeated_dev_gate.py" \
    "${validator_common[@]}" --emit-positive-park-tsv
)
mapfile -t negative_parked_rows < <(
  python3 "${repo_root}/scripts/validate_p3_perception_repeated_dev_gate.py" \
    "${validator_common[@]}" --emit-negative-park-tsv
)
if [[ "${#positive_parked_rows[@]}" -ne 2 ]] || [[ "${#negative_parked_rows[@]}" -ne 2 ]]; then
  echo "each repeated-gate scene must emit exactly two parked models" >&2
  exit 2
fi

park_arguments() {
  local row
  local model target_id x y z
  for row in "$@"; do
    IFS=$'\t' read -r model target_id x y z <<<"${row}"
    printf '%s\0' \
      --park-model-pose "${model}" "${target_id}" "${x}" "${y}" "${z}"
  done
}
mapfile -d '' -t positive_park_arguments < <(
  park_arguments "${positive_parked_rows[@]}"
)
mapfile -d '' -t negative_park_arguments < <(
  park_arguments "${negative_parked_rows[@]}"
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

global_index=0
for mode in negative positive; do
  if [[ "${mode}" == "positive" ]]; then
    count="${positive_trials}"
    scenario="${positive_scenario}"
    target_model="${positive_target_model}"
    target_id="${positive_target_id}"
    target_x="${positive_x}"
    target_y="${positive_y}"
    target_z="${positive_z}"
    trial_park_arguments=("${positive_park_arguments[@]}")
  else
    count="${negative_trials}"
    scenario="${negative_scenario}"
    target_model="${negative_target_model}"
    target_id="${negative_target_id}"
    target_x="${negative_x}"
    target_y="${negative_y}"
    target_z="${negative_z}"
    trial_park_arguments=("${negative_park_arguments[@]}")
  fi

  for trial_index in $(seq 1 "${count}"); do
    global_index=$((global_index + 1))
    printf -v label '%s_%02d' "${mode}" "${trial_index}"
    if [[ "${mode}" == "positive" ]]; then
      domain_offset=$((trial_index - 1))
    else
      domain_offset=$((positive_trials + trial_index - 1))
    fi
    export ROS_DOMAIN_ID=$((base_domain_id + domain_offset))
    launch_log="${output_dir}/${label}_launch.log"
    client_log="${output_dir}/${label}_client.log"
    result_json="${output_dir}/${label}.json"
    echo "[${global_index}/${total_trials}] ${label}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

    setsid ros2 launch strawberry_bringup system.launch.py \
      headless:=true \
      enable_attachment:=true \
      enable_pose_control:=true \
      start_perception:=true \
      start_oracle_provider:=false \
      start_manipulation:=true \
      start_orchestrator:=true \
      model_path:="${model_path}" \
      confidence_threshold:="${confidence_threshold}" \
      perception_detections_topic:="${detections_topic}" \
      perception_target_topic:="${target_topic}" \
      target_source:=perception \
      control_target_topic:="${control_topic}" \
      shadow_enabled:=false \
      >"${launch_log}" 2>&1 &
    launch_pid=$!

    client_arguments=(
      --startup-timeout-sec 75
      --trial-timeout-sec 180
      --control-topic "${control_topic}"
      --detections-topic "${detections_topic}"
      --target-source perception
      --scenario-id "${scenario}_${trial_index}"
      --expected-target-id "${target_id}"
      --target-model-name "${target_model}"
      --target-position-m "${target_x}" "${target_y}" "${target_z}"
      --control-position-tolerance-m 0.03
      "${trial_park_arguments[@]}"
      --output "${result_json}"
    )
    if [[ "${mode}" == "negative" ]]; then
      client_arguments+=(
        --expect-no-pick
        --minimum-detection-frames-after-scene "${minimum_detection_frames}"
      )
    fi
    client_status=0
    timeout --signal=TERM 300 python "${repo_root}/scripts/test_orchestrated_trial.py" \
      "${client_arguments[@]}" \
      >"${client_log}" 2>&1 || client_status=$?
    echo "[${global_index}/${total_trials}] client status ${client_status}"
    shutdown_launch
  done
done

trap - EXIT INT TERM
python "${repo_root}/scripts/summarize_p3_perception_repeated_dev_gate.py" \
  --output-dir "${output_dir}" \
  --manifest "${manifest_path}" \
  --model "${model_path}" \
  --base-domain-id "${base_domain_id}"
