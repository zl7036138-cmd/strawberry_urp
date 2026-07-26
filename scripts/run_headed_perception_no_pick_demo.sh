#!/usr/bin/env bash
set -euo pipefail

# Visible, non-formal only-unripe safety demo under the ADR-0026 engineering
# waiver. It is not a repeated/formal benchmark trial.

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
model_path="${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt"
rviz_config="${repo_root}/config/headed_oracle_shadow_demo.rviz"
output_dir="${1:-${repo_root}/results/development/headed_perception_no_pick_demo_v1}"
domain_id="${2:-216}"
startup_observation_sec="${HEADED_DEMO_STARTUP_OBSERVATION_SEC:-15}"
completion_hold_sec="${HEADED_DEMO_COMPLETION_HOLD_SEC:-12}"
expected_model_sha256="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"

if ! [[ "${domain_id}" =~ ^[0-9]+$ ]] || ((domain_id > 232)); then
  echo "ROS domain ID must be an integer in [0, 232]" >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "WSLg display is unavailable; refusing to silently fall back to headless" >&2
  exit 3
fi
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/build_and_test.sh first." >&2
  exit 4
fi
if [[ ! -f "${model_path}" ]]; then
  echo "ADR-0026 model is missing: ${model_path}" >&2
  exit 5
fi
actual_model_sha256="$(sha256sum "${model_path}" | awk '{print $1}')"
if [[ "${actual_model_sha256}" != "${expected_model_sha256}" ]]; then
  echo "ADR-0026 model hash differs" >&2
  exit 6
fi
if [[ ! -f "${rviz_config}" ]]; then
  echo "RViz configuration is missing: ${rviz_config}" >&2
  exit 7
fi
if [[ -e "${output_dir}" ]]; then
  echo "Output path already exists; choose a new development output directory" >&2
  exit 8
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u

mkdir -p "${output_dir}"
export ROS_DOMAIN_ID="${domain_id}"

launch_pid=""
rviz_pid=""
recorder_pid=""
cleanup() {
  local pid
  for pid in "${recorder_pid}" "${rviz_pid}" "${launch_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 -- "-${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done
  for _ in $(seq 1 120); do
    local alive=false
    for pid in "${recorder_pid}" "${rviz_pid}" "${launch_pid}"; do
      if [[ -n "${pid}" ]] && kill -0 -- "-${pid}" 2>/dev/null; then
        alive=true
      fi
    done
    [[ "${alive}" == "false" ]] && break
    sleep 0.1
  done
  for pid in "${recorder_pid}" "${rviz_pid}" "${launch_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 -- "-${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    [[ -z "${pid}" ]] || wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

cat >"${output_dir}/run_contract.txt" <<EOF
scope=NON_ACCEPTANCE_ONLY_UNRIPE_HEADED_DEMO
control_source=PERCEPTION_ENGINEERING_WAIVER
control_topic=/strawberry/target_pose
model_sha256=${actual_model_sha256}
confidence_threshold=0.58
expected_outcome=NO_PICK
formal_real_test_consumed=false
formal_simulator_matrix_consumed=false
ros_domain_id=${ROS_DOMAIN_ID}
EOF

echo "============================================================"
echo " NON-FORMAL ONLY-UNRIPE HEADED DEMO"
echo " CONTROL = PERCEPTION UNDER ADR-0026 ENGINEERING WAIVER"
echo " EXPECTED = NO_PICK, NO ARM MOTION"
echo "============================================================"

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=false \
  enable_attachment:=true \
  enable_pose_control:=true \
  start_oracle_provider:=false \
  start_manipulation:=true \
  start_orchestrator:=true \
  start_perception:=true \
  target_source:=perception \
  control_target_topic:=/strawberry/target_pose \
  perception_detections_topic:=/strawberry/detections \
  perception_target_topic:=/strawberry/target_pose \
  shadow_enabled:=false \
  model_path:="${model_path}" \
  confidence_threshold:=0.58 \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

setsid python "${repo_root}/scripts/record_p5_headed_demo.py" \
  --output "${output_dir}/camera_overlay_demo.avi" \
  --receipt "${output_dir}/camera_overlay_receipt.json" \
  --detections-topic /strawberry/detections \
  --control-source PERCEPTION \
  --model-role ADR-0026_ENGINEERING-WAIVER \
  >"${output_dir}/recorder.log" 2>&1 &
recorder_pid=$!

setsid rviz2 -d "${rviz_config}" \
  >"${output_dir}/rviz.log" 2>&1 &
rviz_pid=$!

echo "Gazebo and RViz are starting; observing for ${startup_observation_sec}s."
sleep "${startup_observation_sec}"

client_status=0
timeout --signal=TERM 120 python "${repo_root}/scripts/test_orchestrated_trial.py" \
  --startup-timeout-sec 75 \
  --trial-timeout-sec 30 \
  --control-topic /strawberry/target_pose \
  --detections-topic /strawberry/detections \
  --target-source perception \
  --scenario-id headed_single_unripe_no_pick_01 \
  --expected-target-id 2 \
  --target-model-name strawberry_2 \
  --target-position-m 0.42 -0.12 0.52 \
  --park-model-pose strawberry_1 1 0.20 -2.00 1.00 \
  --park-model-pose strawberry_3 3 0.40 -2.00 1.00 \
  --expect-no-pick \
  --minimum-detection-frames-after-scene 10 \
  --output "${output_dir}/trial.json" \
  2>&1 | tee "${output_dir}/client.log" || client_status=${PIPESTATUS[0]}

echo "Only-unripe demo client status: ${client_status}"
echo "Keeping the GUI open for ${completion_hold_sec}s for inspection."
sleep "${completion_hold_sec}"
exit "${client_status}"
