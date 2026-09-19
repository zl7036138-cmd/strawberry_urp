#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/lib/process_group_cleanup.sh"
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}/install/setup.bash"
cd "${repo_root}"

seed="${1:?seed is required}"
scene_dir="${2:?scene directory is required}"
diagnostic_dir="${3:?diagnostic output directory is required}"
scene_stem="generalized_seed_$(printf '%06d' "${seed}")"
model_path="${STRAWBERRY_GENERALIZED_MODEL_PATH:-outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt}"
base_camera_mast_xyz="${STRAWBERRY_BASE_CAMERA_MAST_XYZ:--0.35 0.45 0.05}"
base_camera_xyz="${STRAWBERRY_BASE_CAMERA_XYZ:-0 0 1.00}"
base_camera_rpy="${STRAWBERRY_BASE_CAMERA_RPY:-0 0.543 -0.480}"

if [[ -e "${diagnostic_dir}" ]] && [[ -n "$(find "${diagnostic_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "refusing to overwrite non-empty diagnostic directory: ${diagnostic_dir}" >&2
  exit 2
fi
mkdir -p "${diagnostic_dir}"

setsid ros2 launch strawberry_bringup generalized_harvest.launch.py \
  headless:=true \
  harvest_control_enabled:=false \
  simulation_seed:="${seed}" \
  base_camera_mast_xyz:="${base_camera_mast_xyz}" \
  base_camera_xyz:="${base_camera_xyz}" \
  base_camera_rpy:="${base_camera_rpy}" \
  world_file:="${repo_root}/${scene_dir}/${scene_stem}.sdf" \
  scene_config_file:="${repo_root}/${scene_dir}/${scene_stem}.yaml" \
  model_path:="${repo_root}/${model_path}" \
  device:=0 \
  >"${diagnostic_dir}/launch.log" 2>&1 &
launch_pid=$!
launch_pgid="${launch_pid}"

cleanup() {
  trap - EXIT
  if terminate_process_group "${launch_pgid}" "${launch_pid}" && \
     process_group_stably_absent "${launch_pgid}" 8; then
    printf '{"schema_version":1,"process_group_id":%s,"outcome":"CLEAN"}\n' \
      "${launch_pgid}" >"${diagnostic_dir}/cleanup.json"
    return 0
  fi
  printf '{"schema_version":1,"process_group_id":%s,"outcome":"FAILED"}\n' \
    "${launch_pgid}" >"${diagnostic_dir}/cleanup.json"
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
  if grep -qx "/strawberry_base_perception" <<<"${node_list}" && \
     grep -qx "/strawberry_base_localization" <<<"${node_list}" && \
     ! grep -qx "/strawberry_target_selector" <<<"${node_list}" && \
     ! grep -qx "/strawberry_harvest_orchestrator" <<<"${node_list}"; then
    ready=1
    break
  fi
  process_group_alive "${launch_pgid}" || break
  sleep 0.5
done
if [[ "${ready}" -ne 1 ]]; then
  echo "generalized zero-motion diagnostic runtime did not become ready" >&2
  exit 10
fi

ros2 run strawberry_bringup generalized_truth_isolation_audit \
  --output "${diagnostic_dir}/truth_isolation.json" \
  --timeout 30 \
  --required-node /strawberry_base_localization \
  --required-node /strawberry_wrist_localization \
  --required-node /strawberry_pick_and_place

python scripts/capture_one_rgb_frame.py \
  --output "${diagnostic_dir}/base_rgb.png" \
  --topic /camera/base/color/image_raw \
  --timeout-sec 30

python scripts/diagnose_generalized_rgbd_frame.py \
  --output "${diagnostic_dir}/rgbd.json" \
  --timeout-sec 45 \
  --minimum-detections 1 \
  >"${diagnostic_dir}/diagnostic.stdout.log"

cleanup_status=0
cleanup || cleanup_status=$?
trap - EXIT INT TERM
exit "${cleanup_status}"
