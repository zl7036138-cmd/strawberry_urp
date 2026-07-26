#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/p5/release_smoke_reference_v1}"
environment_role="${2:-reference}"
requested_base_domain_id="${3:-}"
reference_summary="${4:-}"
config_path="${repo_root}/config/p5_release_smoke_v1.json"

if [[ "${environment_role}" != "reference" && "${environment_role}" != "clean" ]]; then
  echo "environment role must be reference or clean" >&2
  exit 2
fi
if [[ "${environment_role}" == "clean" && -z "${reference_summary}" ]]; then
  echo "clean smoke requires the frozen reference summary path" >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi

IFS=$'\t' read -r \
  model_path confidence_threshold detections_topic target_topic control_topic \
  positive_trials negative_trials minimum_detection_frames manifest_base_domain_id \
  maximum_reference_difference < <(
    python3 "${repo_root}/scripts/validate_p5_release_smoke.py" \
      --config "${config_path}" --emit-runtime-tsv
  )
base_domain_id="${requested_base_domain_id:-${manifest_base_domain_id}}"
total_trials=$((positive_trials + negative_trials))
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]] || ((base_domain_id + total_trials - 1 > 232)); then
  echo "base ROS domain must be an integer whose ten-domain range ends at 232 or below" >&2
  exit 2
fi

IFS=$'\t' read -r \
  positive_scenario positive_target_model positive_target_id \
  positive_x positive_y positive_z < <(
    python3 "${repo_root}/scripts/validate_p5_release_smoke.py" \
      --config "${config_path}" --emit-positive-target-tsv
  )
IFS=$'\t' read -r \
  negative_scenario negative_target_model negative_target_id \
  negative_x negative_y negative_z < <(
    python3 "${repo_root}/scripts/validate_p5_release_smoke.py" \
      --config "${config_path}" --emit-negative-target-tsv
  )
mapfile -t positive_parked_rows < <(
  python3 "${repo_root}/scripts/validate_p5_release_smoke.py" \
    --config "${config_path}" --emit-positive-park-tsv
)
mapfile -t negative_parked_rows < <(
  python3 "${repo_root}/scripts/validate_p5_release_smoke.py" \
    --config "${config_path}" --emit-negative-park-tsv
)

park_arguments() {
  local row model target_id x y z
  for row in "$@"; do
    IFS=$'\t' read -r model target_id x y z <<<"${row}"
    printf '%s\0' --park-model-pose "${model}" "${target_id}" "${x}" "${y}" "${z}"
  done
}
mapfile -d '' -t positive_park_arguments < <(park_arguments "${positive_parked_rows[@]}")
mapfile -d '' -t negative_park_arguments < <(park_arguments "${negative_parked_rows[@]}")

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_dir}"
python3 "${repo_root}/scripts/capture_p5_environment.py" \
  --role "${environment_role}" --output "${output_dir}/environment.json" \
  >"${output_dir}/environment_capture.log"

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
    export ROS_DOMAIN_ID=$((base_domain_id + global_index - 1))
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
      "${client_arguments[@]}" >"${client_log}" 2>&1 || client_status=$?
    echo "[${global_index}/${total_trials}] client status ${client_status}"
    shutdown_launch
  done
done

trap - EXIT INT TERM
summary_arguments=(
  --output-dir "${output_dir}"
  --config "${config_path}"
  --environment-role "${environment_role}"
  --base-domain-id "${base_domain_id}"
)
if [[ -n "${reference_summary}" ]]; then
  summary_arguments+=(--reference-summary "${reference_summary}")
fi
python3 "${repo_root}/scripts/summarize_p5_release_smoke.py" "${summary_arguments[@]}"
