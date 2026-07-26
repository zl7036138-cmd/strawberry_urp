#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
colcon_root="${STRAWBERRY_COLCON_ROOT:-${repo_root}/ros2_ws}"

source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set +u
source "${colcon_root}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
exec ros2 launch strawberry_sim sim.launch.py \
  headless:=false \
  enable_attachment:=false
