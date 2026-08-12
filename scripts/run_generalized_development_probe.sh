#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/lib/process_group_cleanup.sh"
source /opt/ros/jazzy/setup.bash
source "${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}/install/setup.bash"
source /opt/strawberry_venv/bin/activate
cd "${repo_root}"

seed="${1:-44012}"
scene_dir="${2:-.codex_tmp/generalized_runtime_candidates_v1}"
run_tag="${3:-development}"
scene_stem="generalized_seed_$(printf '%06d' "${seed}")"
run_dir=".codex_tmp/generalized_harvest_seed_${seed}_${run_tag}"
startup_timeout_sec="${STRAWBERRY_PROBE_STARTUP_TIMEOUT_SEC:-120}"
idle_timeout_sec="${STRAWBERRY_PROBE_IDLE_TIMEOUT_SEC:-180}"
hard_timeout_sec="${STRAWBERRY_PROBE_HARD_TIMEOUT_SEC:-900}"
cleanup_smoke_sec="${STRAWBERRY_PROBE_CLEANUP_SMOKE_SEC:-0}"
if [[ ! "${cleanup_smoke_sec}" =~ ^[0-9]+$ ]]; then
  echo "STRAWBERRY_PROBE_CLEANUP_SMOKE_SEC must be a non-negative integer" >&2
  exit 2
fi

mkdir -p "${run_dir}"
rm -f \
  "${run_dir}/runtime_probe.json" \
  "${run_dir}/runtime_probe.json.tmp" \
  "${run_dir}/cleanup_probe.json"
setsid ros2 launch strawberry_bringup generalized_harvest.launch.py \
  headless:=true \
  simulation_seed:="${seed}" \
  world_file:="${repo_root}/${scene_dir}/${scene_stem}.sdf" \
  scene_config_file:="${repo_root}/${scene_dir}/${scene_stem}.yaml" \
  model_path:="${repo_root}/outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt" \
  device:=0 \
  > "${run_dir}/launch.log" 2>&1 &
launch_pid=$!
# `setsid` makes its child the session and process-group leader, so its PID is
# the only race-free PGID. Querying `ps` immediately after the asynchronous
# spawn can still observe the child before exec/setsid and return this runner's
# own PGID, causing cleanup to signal itself and orphan Gazebo.
launch_pgid="${launch_pid}"

cleanup() {
  trap - EXIT
  if terminate_process_group "${launch_pgid}" "${launch_pid}" && \
     process_group_stably_absent "${launch_pgid}" 8; then
    printf '{"schema_version":1,"process_group_id":%s,"outcome":"CLEAN"}\n' \
      "${launch_pgid}" > "${run_dir}/cleanup_probe.json"
    return 0
  fi
  printf '{"schema_version":1,"process_group_id":%s,"outcome":"FAILED"}\n' \
    "${launch_pgid}" > "${run_dir}/cleanup_probe.json"
  return 1
}

stop_on_signal() {
  trap - INT TERM
  cleanup
  exit 130
}

trap cleanup EXIT
trap stop_on_signal INT TERM

ready=0
for _ in {1..180}; do
  node_list="$(ros2 node list 2>/dev/null || true)"
  observation_type="$(ros2 service type /strawberry/move_to_observation 2>/dev/null || true)"
  evaluation_type="$(ros2 service type /strawberry/evaluate_target 2>/dev/null || true)"
  action_info="$(ros2 action info /strawberry/pick_and_place 2>/dev/null || true)"
  if grep -qx "/strawberry_base_perception" <<<"${node_list}" && \
     grep -qx "/strawberry_wrist_perception" <<<"${node_list}" && \
     grep -qx "/strawberry_base_localization" <<<"${node_list}" && \
     grep -qx "/strawberry_wrist_localization" <<<"${node_list}" && \
     grep -qx "/strawberry_pick_and_place" <<<"${node_list}" && \
     grep -qx "/strawberry_harvest_orchestrator" <<<"${node_list}" && \
     [[ "${observation_type}" == "strawberry_interfaces/srv/MoveToObservation" ]] && \
     [[ "${evaluation_type}" == "strawberry_interfaces/srv/EvaluateTarget" ]] && \
     grep -q "Action servers: 1" <<<"${action_info}"; then
    ready=1
    break
  fi
  process_group_alive "${launch_pgid}" || break
  sleep 0.5
done

if [[ "${ready}" -ne 1 ]]; then
  echo "generalized runtime did not become ready" >&2
  exit 1
fi

if [[ "${cleanup_smoke_sec}" -gt 0 ]]; then
  sleep "${cleanup_smoke_sec}"
  cleanup_status=0
  cleanup || cleanup_status=$?
  trap - EXIT INT TERM
  exit "${cleanup_status}"
fi

# Let the base tracker accumulate the configured minimum observations before
# starting the batch. This is a fixed warm-up, not a truth or target-ID hint.
sleep 12
recorder_status=0
ros2 run strawberry_bringup generalized_development_probe \
  --output "${run_dir}/runtime_probe.json" \
  --startup-timeout "${startup_timeout_sec}" \
  --idle-timeout "${idle_timeout_sec}" \
  --hard-timeout "${hard_timeout_sec}" || recorder_status=$?
cleanup_status=0
cleanup || cleanup_status=$?
trap - EXIT INT TERM
if [[ "${cleanup_status}" -ne 0 ]]; then
  echo "generalized runtime process group did not terminate" >&2
  exit 20
fi
exit "${recorder_status}"
