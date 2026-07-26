#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/t60/oracle_shadow_position_diagnostic_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640/weights/best.pt}"
base_domain_id="${3:-220}"
manifest_path="${4:-${repo_root}/config/t60_oracle_shadow_position_diagnostic.json}"

if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 3
fi
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  echo "base_domain_id must be a non-negative integer" >&2
  exit 2
fi
if ((base_domain_id + 4 > 232)); then
  echo "five diagnostic ROS domain IDs would exceed 232" >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 4
fi

python3 "${repo_root}/scripts/validate_t60_diagnostic_manifest.py" \
  --manifest "${manifest_path}" \
  --model "${model_path}"
mapfile -t scenario_rows < <(
  python3 "${repo_root}/scripts/validate_t60_diagnostic_manifest.py" \
    --manifest "${manifest_path}" \
    --model "${model_path}" \
    --emit-tsv
)
mapfile -t parked_model_rows < <(
  python3 "${repo_root}/scripts/validate_t60_diagnostic_manifest.py" \
    --manifest "${manifest_path}" \
    --model "${model_path}" \
    --emit-park-tsv
)
if [[ "${#scenario_rows[@]}" -ne 5 ]]; then
  echo "validated manifest did not emit exactly five scenarios" >&2
  exit 2
fi

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

park_arguments=()
for row in "${parked_model_rows[@]}"; do
  IFS=$'\t' read -r parked_model parked_target_id parked_x parked_y parked_z <<<"${row}"
  park_arguments+=(
    --park-model-pose
    "${parked_model}"
    "${parked_target_id}"
    "${parked_x}"
    "${parked_y}"
    "${parked_z}"
  )
done

scenario_index=0
for row in "${scenario_rows[@]}"; do
  IFS=$'\t' read -r scenario_id position_label target_model target_id x y z <<<"${row}"
  export ROS_DOMAIN_ID=$((base_domain_id + scenario_index))
  launch_log="${output_dir}/${scenario_id}_launch.log"
  client_log="${output_dir}/${scenario_id}_client.log"
  result_json="${output_dir}/${scenario_id}.json"
  echo "[$((scenario_index + 1))/5] ${scenario_id} (${position_label}), ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

  setsid ros2 launch strawberry_bringup system.launch.py \
    headless:=true \
    enable_attachment:=true \
    enable_pose_control:=true \
    start_oracle_provider:=true \
    start_manipulation:=true \
    start_orchestrator:=true \
    start_perception:=true \
    target_source:=oracle \
    control_target_topic:=/strawberry/oracle/target_pose \
    oracle_target_id:="${target_id}" \
    shadow_enabled:=true \
    model_path:="${model_path}" \
    confidence_threshold:=0.31 \
    >"${launch_log}" 2>&1 &
  launch_pid=$!

  client_status=0
  timeout --signal=TERM 260 python "${repo_root}/scripts/test_orchestrated_trial.py" \
    --startup-timeout-sec 75 \
    --trial-timeout-sec 180 \
    --scenario-id "${scenario_id}" \
    --expected-target-id "${target_id}" \
    --target-model-name "${target_model}" \
    --target-position-m "${x}" "${y}" "${z}" \
    "${park_arguments[@]}" \
    --expect-shadow \
    --output "${result_json}" \
    >"${client_log}" 2>&1 || client_status=$?
  echo "[$((scenario_index + 1))/5] client status ${client_status}"
  shutdown_launch
  scenario_index=$((scenario_index + 1))
done

trap - EXIT INT TERM
python "${repo_root}/scripts/summarize_t60_position_diagnostic.py" \
  --output-dir "${output_dir}" \
  --manifest "${manifest_path}" \
  --model "${model_path}" \
  --base-domain-id "${base_domain_id}"
