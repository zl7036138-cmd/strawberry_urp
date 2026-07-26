#!/usr/bin/env bash
set -euo pipefail

# Visible, non-acceptance development demo.  Robot motion is controlled only by
# Gazebo truth; the rejected simulator-adaptation model remains shadow-only.

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
model_path="${repo_root}/outputs/perception/yolo11s_640_sim_adapt_v1/weights/best.pt"
rviz_config="${repo_root}/config/headed_oracle_shadow_demo.rviz"
output_dir="${1:-${repo_root}/results/development/headed_oracle_shadow_demo_v1}"
domain_id="${2:-223}"
startup_observation_sec="${HEADED_DEMO_STARTUP_OBSERVATION_SEC:-12}"
completion_hold_sec="${HEADED_DEMO_COMPLETION_HOLD_SEC:-15}"
record_video="${HEADED_DEMO_RECORD_VIDEO:-true}"
expected_model_sha256="2806968b8b4157169302948a75c99427de41b287b9c77dd2cfd4a436782c7e17"

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
  echo "Rejected shadow candidate is missing: ${model_path}" >&2
  exit 5
fi
actual_model_sha256="$(sha256sum "${model_path}" | awk '{print $1}')"
if [[ "${actual_model_sha256}" != "${expected_model_sha256}" ]]; then
  echo "Rejected shadow candidate hash differs from ADR 0022" >&2
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
  for _ in $(seq 1 100); do
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
scope=NON_ACCEPTANCE_HEADED_DEVELOPMENT_DEMO
control_source=ORACLE
control_topic=/strawberry/oracle/target_pose
shadow_model_role=REJECTED_CANDIDATE_DO_NOT_PROMOTE
shadow_model_sha256=${actual_model_sha256}
shadow_confidence_threshold=0.80
camera_overlay_recording=${record_video}
formal_real_test_consumed=false
formal_simulator_matrix_consumed=false
ros_domain_id=${ROS_DOMAIN_ID}
EOF

echo "============================================================"
echo " NON-FORMAL HEADED DEVELOPMENT DEMO"
echo " CONTROL = ORACLE (/strawberry/oracle/target_pose)"
echo " MODEL   = REJECTED CANDIDATE, SHADOW ONLY"
echo "============================================================"

setsid ros2 launch strawberry_bringup system.launch.py \
  headless:=false \
  enable_attachment:=true \
  enable_pose_control:=true \
  start_oracle_provider:=true \
  start_manipulation:=true \
  start_orchestrator:=true \
  start_perception:=true \
  target_source:=oracle \
  control_target_topic:=/strawberry/oracle/target_pose \
  oracle_target_id:=1 \
  shadow_enabled:=true \
  model_path:="${model_path}" \
  confidence_threshold:=0.80 \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

if [[ "${record_video}" == "true" ]]; then
  setsid python "${repo_root}/scripts/record_p5_headed_demo.py" \
    --output "${output_dir}/camera_overlay_demo.avi" \
    --receipt "${output_dir}/camera_overlay_receipt.json" \
    >"${output_dir}/recorder.log" 2>&1 &
  recorder_pid=$!
fi

setsid rviz2 -d "${rviz_config}" \
  >"${output_dir}/rviz.log" 2>&1 &
rviz_pid=$!

echo "Gazebo and RViz are starting; the arm will remain still for ${startup_observation_sec}s."
sleep "${startup_observation_sec}"

client_status=0
timeout --signal=TERM 260 python "${repo_root}/scripts/test_orchestrated_trial.py" \
  --startup-timeout-sec 75 \
  --trial-timeout-sec 180 \
  --scenario-id headed_single_clear_position_01 \
  --expected-target-id 1 \
  --target-model-name strawberry_1 \
  --target-position-m 0.44 -0.10 0.48 \
  --park-model-pose strawberry_2 2 0.20 -2.00 1.00 \
  --park-model-pose strawberry_3 3 0.40 -2.00 1.00 \
  --expect-shadow \
  --output "${output_dir}/trial.json" \
  2>&1 | tee "${output_dir}/client.log" || client_status=${PIPESTATUS[0]}

echo "Development trial client status: ${client_status}"
echo "Keeping the GUI open for ${completion_hold_sec}s for inspection."
sleep "${completion_hold_sec}"
exit "${client_status}"
