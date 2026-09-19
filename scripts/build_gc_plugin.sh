#!/usr/bin/env bash
# Build the gravity-compensating gz_ros2_control plugin (upstream branch
# add/gravity_compensation) into ~/gzrc_gc. Sets STRAWBERRY_GC_PLUGIN_LIB_DIR
# for the current shell; sim.launch.py picks it up via GZ_SIM_SYSTEM_PLUGIN_PATH.
# The compensated plugin adds the measured load torque (gravity + attached
# fruit) to every effort command - the simulation-side setLoad semantics
# (see docs/p0/CONTROL_EXPERIMENT_CAMPAIGN_2026-09-16.md).
set -euo pipefail
SRC_DIR="${HOME}/gzrc_gc"
if [[ ! -d "${SRC_DIR}" ]]; then
  git clone --depth 1 -b add/gravity_compensation \
    https://github.com/ros-controls/gz_ros2_control.git "${SRC_DIR}"
fi
source /opt/ros/jazzy/setup.bash
cd "${SRC_DIR}"
colcon build --packages-select gz_ros2_control \
  --build-base "${SRC_DIR}/build" \
  --install-base "${SRC_DIR}/install"
LIB_DIR="${SRC_DIR}/install/gz_ros2_control/lib"
export STRAWBERRY_GC_PLUGIN_LIB_DIR="${LIB_DIR}"
echo "export STRAWBERRY_GC_PLUGIN_LIB_DIR=${LIB_DIR}"
