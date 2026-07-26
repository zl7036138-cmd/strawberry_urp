#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${repo_root}/ros2_ws"
# Native Linux storage avoids chmod / atomic-replace failures in Python and
# CMake tooling on a DrvFs-mounted Windows workspace.  Override this path when
# a separate build tree is desired.
artifact_root="${STRAWBERRY_COLCON_ROOT:-${HOME}/.cache/strawberry_urp/colcon}"
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
# The apt-installed ``colcon`` entry point has a /usr/bin/python3 shebang.
# Calling it directly for the *build* would make ament_python console scripts
# inherit the system interpreter, which cannot import the Ultralytics stack
# installed only in /opt/strawberry_venv.  Execute the build entry point with
# the active virtual-environment interpreter so runtime nodes receive the
# matching shebang.  Keep test and test-result under the apt entry point's own
# interpreter: colcon's Python-package test extension otherwise searches the
# isolated build directory and reports zero discovered tests in this venv.
colcon_python="${VIRTUAL_ENV}/bin/python"
colcon_executable="$(command -v colcon)"
set -u
cd "${workspace_root}"
mkdir -p "${artifact_root}/build" "${artifact_root}/install" "${artifact_root}/log"

# rosdep keeps its resolved source index in each user's home directory.  The
# bootstrap script is intentionally run as root, while normal builds are not,
# so initialise the current user's cache on first use.  Prefer the pinned local
# snapshot installed by bootstrap to keep this step reproducible and offline.
if [[ ! -s "${HOME}/.ros/rosdep/sources.cache/index" ]]; then
  if [[ -f /etc/ros/rosdep/local-sources/index-v4.yaml ]]; then
    export ROSDISTRO_INDEX_URL=file:///etc/ros/rosdep/local-sources/index-v4.yaml
  fi
  rosdep update --rosdistro jazzy
fi

rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
"${colcon_python}" "${colcon_executable}" \
  --log-base "${artifact_root}/log" build \
  --base-paths src \
  --build-base "${artifact_root}/build" \
  --install-base "${artifact_root}/install" \
  --symlink-install \
  --event-handlers console_cohesion+

# Fail the build immediately if a future invocation accidentally falls back to
# the system colcon interpreter.  That regression is otherwise invisible to
# unit tests and surfaces only when the perception node imports Ultralytics.
perception_entry="${artifact_root}/install/strawberry_perception/lib/strawberry_perception/perception_node"
expected_shebang="#!${colcon_python}"
if [[ ! -f "${perception_entry}" ]]; then
  echo "Missing installed perception entry point: ${perception_entry}" >&2
  exit 5
fi
IFS= read -r actual_shebang < "${perception_entry}"
if [[ "${actual_shebang}" != "${expected_shebang}" ]]; then
  echo "Perception entry point uses ${actual_shebang}; expected ${expected_shebang}." >&2
  exit 5
fi

set +u
source "${artifact_root}/install/setup.bash"
set -u
"${colcon_executable}" \
  --log-base "${artifact_root}/log" test \
  --base-paths src \
  --build-base "${artifact_root}/build" \
  --install-base "${artifact_root}/install" \
  --test-result-base "${artifact_root}/build" \
  --event-handlers console_cohesion+ \
  --return-code-on-test-failure
"${colcon_executable}" test-result \
  --test-result-base "${artifact_root}/build" --verbose
