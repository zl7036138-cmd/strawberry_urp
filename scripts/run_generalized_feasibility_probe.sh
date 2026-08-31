#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/lib/process_group_cleanup.sh"
source /opt/ros/jazzy/setup.bash
source "${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}/install/setup.bash"
source /opt/strawberry_venv/bin/activate
cd "${repo_root}"

seed="${1:?seed is required}"
scene_dir="${2:?scene directory is required}"
run_tag="${3:-zero_motion}"
scene_stem="generalized_seed_$(printf '%06d' "${seed}")"
run_dir="${STRAWBERRY_FEASIBILITY_OUTPUT_DIR:-.codex_tmp/generalized_feasibility_seed_${seed}_${run_tag}}"
model_path="${STRAWBERRY_GENERALIZED_MODEL_PATH:-outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt}"
mkdir -p "${run_dir}"
for output in feasibility_probe.json observability.json truth_isolation.json cleanup_probe.json; do
  if [[ -e "${run_dir}/${output}" ]]; then
    echo "refusing to overwrite ${run_dir}/${output}" >&2
    exit 2
  fi
done

setsid ros2 launch strawberry_bringup generalized_harvest.launch.py \
  headless:=true \
  harvest_control_enabled:=false \
  simulation_seed:="${seed}" \
  world_file:="${repo_root}/${scene_dir}/${scene_stem}.sdf" \
  scene_config_file:="${repo_root}/${scene_dir}/${scene_stem}.yaml" \
  model_path:="${repo_root}/${model_path}" \
  device:=0 \
  >"${run_dir}/launch.log" 2>&1 &
launch_pid=$!
launch_pgid="${launch_pid}"

cleanup() {
  trap - EXIT
  if terminate_process_group "${launch_pgid}" "${launch_pid}" && \
     process_group_stably_absent "${launch_pgid}" 8; then
    printf '{"schema_version":1,"process_group_id":%s,"outcome":"CLEAN"}\n' \
      "${launch_pgid}" >"${run_dir}/cleanup_probe.json"
    return 0
  fi
  printf '{"schema_version":1,"process_group_id":%s,"outcome":"FAILED"}\n' \
    "${launch_pgid}" >"${run_dir}/cleanup_probe.json"
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
for _ in {1..240}; do
  node_list="$(ros2 node list 2>/dev/null || true)"
  evaluation_type="$(ros2 service type /strawberry/evaluate_target 2>/dev/null || true)"
  if grep -qx "/strawberry_base_perception" <<<"${node_list}" && \
     grep -qx "/strawberry_wrist_perception" <<<"${node_list}" && \
     grep -qx "/strawberry_base_localization" <<<"${node_list}" && \
     grep -qx "/strawberry_wrist_localization" <<<"${node_list}" && \
     grep -qx "/strawberry_pick_and_place" <<<"${node_list}" && \
     ! grep -qx "/strawberry_target_selector" <<<"${node_list}" && \
     ! grep -qx "/strawberry_harvest_orchestrator" <<<"${node_list}" && \
     [[ "${evaluation_type}" == "strawberry_interfaces/srv/EvaluateTarget" ]]; then
    ready=1
    break
  fi
  process_group_alive "${launch_pgid}" || break
  sleep 0.5
done

if [[ "${ready}" -ne 1 ]]; then
  echo "generalized zero-motion runtime did not become ready" >&2
  exit 10
fi

audit_status=0
ros2 run strawberry_bringup generalized_truth_isolation_audit \
  --output "${run_dir}/truth_isolation.json" \
  --timeout 30 \
  --required-node /strawberry_base_localization \
  --required-node /strawberry_wrist_localization \
  --required-node /strawberry_pick_and_place || audit_status=$?
if [[ "${audit_status}" -ne 0 ]]; then
  echo "truth-isolation audit failed" >&2
  exit 11
fi

observability_status=0
python scripts/diagnose_generalized_rgbd_frame.py \
  --output "${run_dir}/observability.json" \
  --timeout-sec 45 \
  --minimum-detections 0 \
  >"${run_dir}/observability.stdout.log" || observability_status=$?
if [[ "${observability_status}" -ne 0 ]]; then
  echo "base-camera observability diagnostic failed" >&2
  exit 12
fi

probe_status=0
ros2 run strawberry_bringup generalized_feasibility_probe \
  --output "${run_dir}/feasibility_probe.json" \
  --scene-config "${repo_root}/${scene_dir}/${scene_stem}.yaml" \
  --observation-window "${STRAWBERRY_FEASIBILITY_OBSERVATION_WINDOW_SEC:-15}" \
  --startup-timeout "${STRAWBERRY_FEASIBILITY_STARTUP_TIMEOUT_SEC:-90}" \
  --evaluation-timeout "${STRAWBERRY_FEASIBILITY_EVALUATION_TIMEOUT_SEC:-30}" \
  --hard-timeout "${STRAWBERRY_FEASIBILITY_HARD_TIMEOUT_SEC:-240}" \
  --minimum-feasible-ripe 2 || probe_status=$?

cleanup_status=0
cleanup || cleanup_status=$?
trap - EXIT INT TERM
if [[ "${cleanup_status}" -ne 0 ]]; then
  exit 20
fi
exit "${probe_status}"
