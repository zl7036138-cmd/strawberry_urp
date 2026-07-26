#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"
output_dir="${1:-${repo_root}/results/development/blender_scene_v2_single_fruit_60f_v1}"
model_path="${2:-${repo_root}/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt}"
base_domain_id="${3:-228}"
base_world="${repo_root}/ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"
expected_sha="e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
[[ -f "${artifact_root}/install/setup.bash" ]] || { echo "Workspace is not built" >&2; exit 3; }
[[ "$(sha256sum "${model_path}" | awk '{print $1}')" == "${expected_sha}" ]] || { echo "Wrong model hash" >&2; exit 2; }
[[ ! -e "${output_dir}" ]] || { echo "Output exists; choose a fresh directory" >&2; exit 4; }
[[ "${base_domain_id}" =~ ^[0-9]+$ ]] && ((base_domain_id+2<=232)) || { echo "Three domain IDs must fit [0,232]" >&2; exit 2; }
set +u
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "${artifact_root}/install/setup.bash"
set -u
mkdir -p "${output_dir}"; launch_pid=""
shutdown_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 150); do kill -0 -- "-${launch_pid}" 2>/dev/null || break; sleep .1; done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true; wait "${launch_pid}" 2>/dev/null || true
  fi
  launch_pid=""
}
trap shutdown_launch EXIT INT TERM
index=0
for target in strawberry_1 strawberry_3 strawberry_2; do
  d="${output_dir}/${target}"; mkdir -p "${d}"
  python3 "${repo_root}/scripts/materialize_blender_v2_single_fruit.py" --base-world "${base_world}" \
    --target "${target}" --output-world "${d}/world.sdf" --output-receipt "${d}/scene_receipt.json"
  export ROS_DOMAIN_ID=$((base_domain_id+index))
  setsid ros2 launch strawberry_bringup system.launch.py headless:=true world_file:="${d}/world.sdf" \
    start_perception:=true start_oracle_provider:=false start_manipulation:=false start_orchestrator:=false \
    enable_attachment:=false enable_pose_control:=false model_path:="${model_path}" confidence_threshold:=0.58 \
    perception_detections_topic:=/strawberry/shadow/detections perception_target_topic:=/strawberry/shadow/target_pose \
    >"${d}/launch.log" 2>&1 & launch_pid=$!
  status=0
  timeout --signal=TERM 180 ros2 run strawberry_perception shadow_window_probe --output-json "${d}/shadow_window.json" \
    --scenario-id "blender_v2_single_${target}" --lighting nominal --occlusion natural_plant \
    --warmup-frames 10 --frames 60 --timeout-sec 120 --post-window-wait-sec 2 \
    --window-boundary after_model_warmup_before_robot_motion >"${d}/shadow_window.log" 2>&1 || status=$?
  shutdown_launch
  [[ "${status}" -eq 0 ]] || { echo "${target} failed: ${status}" >&2; exit 1; }
  index=$((index+1))
done
trap - EXIT INT TERM
python3 "${repo_root}/scripts/summarize_blender_v2_single_fruit_diagnostic.py" --output-dir "${output_dir}"
