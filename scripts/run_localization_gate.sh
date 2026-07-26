#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runner_script="$(realpath "${BASH_SOURCE[0]}")"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
output_dir="${1:-${repo_root}/results/p2/localization_gate}"
launch_log="${output_dir}/launch.log"
gate_log="${output_dir}/gate.log"
result_json="${output_dir}/summary.json"
result_csv="${output_dir}/samples.csv"

source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set -u
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${repo_root}/scripts/build_and_test.sh first." >&2
  exit 3
fi
set +u
source "${artifact_root}/install/setup.bash"
set -u
attempts_per_position="${STRAWBERRY_LOCALIZATION_ATTEMPTS_PER_POSITION:-3}"
sensor_timeout_sec="${STRAWBERRY_LOCALIZATION_SENSOR_TIMEOUT_SEC:-5.0}"
runner_timeout_sec="${STRAWBERRY_LOCALIZATION_RUNNER_TIMEOUT_SEC:-360}"
domain_probe_spin_sec="${STRAWBERRY_LOCALIZATION_DOMAIN_PROBE_SPIN_SEC:-1.0}"

normalize_domain() {
  local raw_domain="$1"
  if [[ ! "${raw_domain}" =~ ^[0-9]+$ ]]; then
    echo "ROS domain must be an integer from 0 through 232: ${raw_domain}" >&2
    return 1
  fi
  local decimal_domain=$((10#${raw_domain}))
  if (( decimal_domain > 232 )); then
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
    echo "ROS domain ${candidate} preflight failed (status ${probe_status})." >&2
    return 2
  }
  local node_name
  while IFS= read -r node_name; do
    node_name="${node_name%$'\r'}"
    [[ -z "${node_name}" ]] && continue
    # ros2 creates this short-lived hidden node for a --no-daemon query.
    [[ "${node_name}" =~ ^/_ros2cli_[0-9]+$ ]] && continue
    DOMAIN_PROBE_NODES+="${node_name} "
  done <<< "${node_output}"
  [[ -z "${DOMAIN_PROBE_NODES}" ]]
}

# The PID only selects where to begin searching.  Isolation is established by
# a direct DDS graph query without the ROS daemon.  This is a preflight check,
# not an atomic reservation; the selected domain is recorded in the summary.
requested_domain="${ROS_DOMAIN_ID:-${STRAWBERRY_LOCALIZATION_DOMAIN_ID:-}}"
if [[ -n "${requested_domain}" ]]; then
  selected_domain="$(normalize_domain "${requested_domain}")" || exit 3
  domain_selection_mode="explicit"
  if domain_is_empty "${selected_domain}"; then
    :
  else
    probe_status=$?
    if (( probe_status == 1 )); then
      echo "ROS domain ${selected_domain} already has nodes: ${DOMAIN_PROBE_NODES}" >&2
    fi
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
    if (( probe_status == 2 )); then
      exit 5
    fi
  done
  if [[ -z "${selected_domain}" ]]; then
    echo "No empty ROS domain was found in the automatic range 100-199." >&2
    exit 5
  fi
fi
export ROS_DOMAIN_ID="${selected_domain}"

if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  echo "Choose a new directory so prior acceptance evidence is preserved." >&2
  exit 4
fi
mkdir -p "${output_dir}"

launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 150); do
      if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
      sleep 1
    fi
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    fi
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap shutdown_launch EXIT INT TERM

setsid ros2 launch strawberry_localization localization_gate.launch.py \
  headless:=true >"${launch_log}" 2>&1 &
launch_pid=$!

gate_status=0
timeout --signal=TERM "${runner_timeout_sec}" \
  ros2 run strawberry_localization localization_gate \
    --output-json "${result_json}" \
    --output-csv "${result_csv}" \
    --attempts-per-position "${attempts_per_position}" \
    --sensor-timeout-sec "${sensor_timeout_sec}" \
    --runner-timeout-sec "${runner_timeout_sec}" \
    --launch-headless \
    --runner-script "${runner_script}" \
    --domain-selection-mode "${domain_selection_mode}" \
    --domain-probe-spin-sec "${domain_probe_spin_sec}" \
  >"${gate_log}" 2>&1 || gate_status=$?

shutdown_launch
trap - EXIT INT TERM

if [[ ! -s "${result_json}" ]] || [[ ! -s "${result_csv}" ]]; then
  echo "Localization gate did not produce complete result artifacts." >&2
  gate_status=1
fi
if grep -Eqi "Traceback|exception was never retrieved|process has died" \
  "${launch_log}" "${gate_log}"; then
  echo "Localization gate logs contain an unhandled runtime failure." >&2
  gate_status=1
fi

echo "Localization result: ${result_json}"
echo "Localization samples: ${result_csv}"
echo "ROS domain: ${ROS_DOMAIN_ID}"
echo "Launch log: ${launch_log}"
echo "Gate log: ${gate_log}"
exit "${gate_status}"
