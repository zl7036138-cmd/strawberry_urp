#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/p3/perception_control_waiver_smoke_v1}"
ros_domain_id="${2:-228}"
manifest_path="${3:-${repo_root}/config/p3_perception_control_waiver_v1.json}"
model_override="${4:-}"

if ! [[ "${ros_domain_id}" =~ ^[0-9]+$ ]] || ((ros_domain_id > 232)); then
  echo "ros_domain_id must be an integer in [0, 232]" >&2
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

validator_common=(--manifest "${manifest_path}")
if [[ -n "${model_override}" ]]; then
  validator_common+=(--model "${model_override}")
fi
IFS=$'\t' read -r model_path confidence_threshold detections_topic target_topic control_topic < <(
  python3 "${repo_root}/scripts/validate_p3_perception_control_waiver.py" \
    "${validator_common[@]}" \
    --emit-runtime-tsv
)
IFS=$'\t' read -r scenario_id target_model target_id target_x target_y target_z < <(
  python3 "${repo_root}/scripts/validate_p3_perception_control_waiver.py" \
    "${validator_common[@]}" \
    --emit-target-tsv
)
mapfile -t parked_rows < <(
  python3 "${repo_root}/scripts/validate_p3_perception_control_waiver.py" \
    "${validator_common[@]}" \
    --emit-park-tsv
)
if [[ "${#parked_rows[@]}" -ne 2 ]]; then
  echo "waiver manifest must emit exactly two parked models" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_dir}"
export ROS_DOMAIN_ID="${ros_domain_id}"

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
for row in "${parked_rows[@]}"; do
  IFS=$'\t' read -r parked_model parked_id parked_x parked_y parked_z <<<"${row}"
  park_arguments+=(
    --park-model-pose
    "${parked_model}"
    "${parked_id}"
    "${parked_x}"
    "${parked_y}"
    "${parked_z}"
  )
done

launch_log="${output_dir}/trial_01_launch.log"
client_log="${output_dir}/trial_01_client.log"
result_json="${output_dir}/trial_01.json"
echo "Perception-control waiver smoke: ${scenario_id}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
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

client_status=0
timeout --signal=TERM 300 python "${repo_root}/scripts/test_orchestrated_trial.py" \
  --startup-timeout-sec 75 \
  --trial-timeout-sec 180 \
  --control-topic "${control_topic}" \
  --target-source perception \
  --scenario-id "${scenario_id}" \
  --expected-target-id "${target_id}" \
  --target-model-name "${target_model}" \
  --target-position-m "${target_x}" "${target_y}" "${target_z}" \
  --control-position-tolerance-m 0.03 \
  "${park_arguments[@]}" \
  --output "${result_json}" \
  >"${client_log}" 2>&1 || client_status=$?
echo "Perception-control client status ${client_status}"
shutdown_launch
trap - EXIT INT TERM

summary_arguments=(
  --output-dir "${output_dir}"
  --manifest "${manifest_path}"
  --model "${model_path}"
  --ros-domain-id "${ros_domain_id}"
)
python "${repo_root}/scripts/summarize_p3_perception_control_smoke.py" \
  "${summary_arguments[@]}"
