#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runner_script="$(realpath "${BASH_SOURCE[0]}")"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
contract="${1:-${repo_root}/config/blender_v2_localization_accuracy_100_v1.json}"
contract="$(realpath "${contract}")"
relative_output="$(
  python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["execution"]["output_directory"])' \
    "${contract}"
)"
output_dir="${repo_root}/${relative_output}"
world_file="${output_dir}/camera_clear_world.sdf"
world_receipt="${output_dir}/camera_clear_world_receipt.json"
launch_log="${output_dir}/launch.log"
gate_log="${output_dir}/gate.log"
result_json="${output_dir}/summary.json"
result_csv="${output_dir}/samples.csv"

[[ -f "${artifact_root}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${artifact_root}" >&2
  exit 3
}
python3 "${repo_root}/scripts/validate_blender_v2_localization_gate.py" \
  --contract "${contract}"
if [[ -e "${output_dir}" ]]; then
  echo "Frozen output already exists; the contract does not authorize a retry:" >&2
  echo "${output_dir}" >&2
  exit 4
fi
mkdir -p "${output_dir}"
python3 \
  "${repo_root}/scripts/materialize_blender_v2_localization_gate_world.py" \
  --contract "${contract}" \
  --output-world "${world_file}" \
  --output-receipt "${world_receipt}" \
  >"${output_dir}/materialization.log"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u

domain_probe_spin_sec=1.0
normalize_domain() {
  local raw_domain="$1"
  if [[ ! "${raw_domain}" =~ ^[0-9]+$ ]]; then
    echo "ROS domain must be an integer from 0 through 232: ${raw_domain}" >&2
    return 1
  fi
  local decimal_domain=$((10#${raw_domain}))
  if ((decimal_domain > 232)); then
    echo "ROS domain must be an integer from 0 through 232: ${raw_domain}" >&2
    return 1
  fi
  printf '%s' "${decimal_domain}"
}

domain_is_empty() {
  local candidate="$1"
  local node_output
  local probe_status
  DOMAIN_PROBE_NODES=""
  node_output="$(
    ROS_DOMAIN_ID="${candidate}" timeout --signal=TERM 5 \
      ros2 node list --no-daemon --spin-time "${domain_probe_spin_sec}" --all
  )" || {
    probe_status=$?
    echo "ROS domain ${candidate} preflight failed (${probe_status})." >&2
    return 2
  }
  local node_name
  while IFS= read -r node_name; do
    node_name="${node_name%$'\r'}"
    [[ -z "${node_name}" ]] && continue
    [[ "${node_name}" =~ ^/_ros2cli_[0-9]+$ ]] && continue
    DOMAIN_PROBE_NODES+="${node_name} "
  done <<<"${node_output}"
  [[ -z "${DOMAIN_PROBE_NODES}" ]]
}

requested_domain="${ROS_DOMAIN_ID:-${STRAWBERRY_V2_LOCALIZATION_DOMAIN_ID:-}}"
if [[ -n "${requested_domain}" ]]; then
  selected_domain="$(normalize_domain "${requested_domain}")" || exit 3
  domain_selection_mode="explicit"
  if ! domain_is_empty "${selected_domain}"; then
    echo "ROS domain ${selected_domain} is not empty: ${DOMAIN_PROBE_NODES}" >&2
    exit 5
  fi
else
  domain_selection_mode="automatic"
  first_candidate=$((100 + ($$ % 100)))
  selected_domain=""
  for ((offset = 0; offset < 100; offset++)); do
    candidate=$((100 + ((first_candidate - 100 + offset) % 100)))
    if domain_is_empty "${candidate}"; then
      selected_domain="${candidate}"
      break
    fi
    probe_status=$?
    ((probe_status == 2)) && exit 5
  done
  [[ -n "${selected_domain}" ]] || {
    echo "No empty ROS domain was found in 100-199." >&2
    exit 5
  }
fi
export ROS_DOMAIN_ID="${selected_domain}"

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

setsid ros2 launch strawberry_localization \
  blender_v2_localization_gate.launch.py \
  headless:=true world_file:="${world_file}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

gate_status=0
timeout --signal=TERM 360 \
  ros2 run strawberry_localization localization_gate \
    --output-json "${result_json}" \
    --output-csv "${result_csv}" \
    --attempts-per-position 3 \
    --sensor-timeout-sec 5.0 \
    --runner-timeout-sec 360.0 \
    --launch-headless \
    --runner-script "${runner_script}" \
    --domain-selection-mode "${domain_selection_mode}" \
    --domain-probe-spin-sec "${domain_probe_spin_sec}" \
    --gate-contract "${contract}" \
    --repository-root "${repo_root}" \
    --materialized-world "${world_file}" \
    --world-receipt "${world_receipt}" \
  >"${gate_log}" 2>&1 || gate_status=$?

shutdown_launch
trap - EXIT INT TERM

if [[ ! -s "${result_json}" ]] || [[ ! -s "${result_csv}" ]]; then
  echo "Blender-v2 localization gate did not produce complete artifacts." >&2
  gate_status=1
fi
if grep -Eqi \
  "Traceback|exception was never retrieved|process has died|RuntimeError" \
  "${launch_log}" "${gate_log}"; then
  echo "Blender-v2 localization logs contain a runtime failure." >&2
  gate_status=1
fi

echo "Blender-v2 localization result: ${result_json}"
echo "Blender-v2 localization samples: ${result_csv}"
echo "ROS domain: ${ROS_DOMAIN_ID}"
echo "Launch log: ${launch_log}"
echo "Gate log: ${gate_log}"
exit "${gate_status}"
