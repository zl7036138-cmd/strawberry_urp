#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
set -u
set +u
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${repo_root}/scripts/build_and_test.sh first." >&2
  exit 3
fi
source "${artifact_root}/install/setup.bash"
set -u

exec ros2 launch strawberry_bringup system.launch.py "$@"
