#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run this script as root inside the new Ubuntu-24.04 WSL distribution." >&2
  exit 2
fi

source /etc/os-release
if [[ "${VERSION_ID}" != "24.04" ]]; then
  echo "Ubuntu 24.04 is required; found ${PRETTY_NAME}." >&2
  exit 2
fi

export DEBIAN_FRONTEND=noninteractive
repo_root="${STRAWBERRY_REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
apt-get update
apt-get install -y \
  ca-certificates \
  curl \
  git \
  gnupg \
  locales \
  lsb-release \
  python3-pip \
  python3-venv \
  software-properties-common

locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository -y universe

tmp_deb="$(mktemp --suffix=.deb)"
ros_apt_version="${ROS_APT_SOURCE_VERSION:-1.2.0}"
ros_apt_filename="ros2-apt-source_${ros_apt_version}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb"
cached_ros_apt="${repo_root}/.cache/${ros_apt_filename}"
if [[ -f "${cached_ros_apt}" ]]; then
  cp "${cached_ros_apt}" "${tmp_deb}"
else
  curl --fail --silent --show-error --location \
    --retry 10 --retry-all-errors --retry-delay 2 \
    --connect-timeout 20 --max-time 300 \
    -o "${tmp_deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/${ros_apt_filename}"
fi
dpkg -i "${tmp_deb}"
rm -f "${tmp_deb}"
apt-get update

apt-get install -y \
  fontconfig \
  fonts-open-sans \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-cv-bridge \
  ros-jazzy-desktop \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-moveit-py \
  ros-jazzy-moveit-resources-panda-description \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers

# Ultralytics 8.4.92 otherwise attempts a network download for Arial.ttf when
# it creates plots.  A filename alias to the packaged Open Sans font satisfies
# its lookup without adding an unpinned runtime artifact.
install -d -m 0755 /usr/local/share/fonts/truetype
ln -sfn \
  /usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf \
  /usr/local/share/fonts/truetype/Arial.ttf
fc-cache -f

rosdep_cache="${repo_root}/.cache/rosdep"
rosdep_local_root="/etc/ros/rosdep/local-sources"
rosdep_local=false
rosdep_cache_files=(
  base.yaml
  python.yaml
  ruby.yaml
  index-v4.yaml
  jazzy-distribution.yaml
)
rosdep_cache_complete=true
for filename in "${rosdep_cache_files[@]}"; do
  if [[ ! -s "${rosdep_cache}/${filename}" ]]; then
    rosdep_cache_complete=false
    break
  fi
done

if [[ "${rosdep_cache_complete}" == "true" ]]; then
  install -d -m 0755 "${rosdep_local_root}/jazzy"
  install -m 0644 "${rosdep_cache}/base.yaml" "${rosdep_local_root}/base.yaml"
  install -m 0644 "${rosdep_cache}/python.yaml" "${rosdep_local_root}/python.yaml"
  install -m 0644 "${rosdep_cache}/ruby.yaml" "${rosdep_local_root}/ruby.yaml"
  install -m 0644 "${rosdep_cache}/index-v4.yaml" "${rosdep_local_root}/index-v4.yaml"
  install -m 0644 \
    "${rosdep_cache}/jazzy-distribution.yaml" \
    "${rosdep_local_root}/jazzy/distribution.yaml"
  install -D -m 0644 \
    "${repo_root}/config/rosdep/20-local.list" \
    /etc/ros/rosdep/sources.list.d/20-default.list
  export ROSDISTRO_INDEX_URL="file://${rosdep_local_root}/index-v4.yaml"
  rosdep_local=true
elif [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

rosdep_updated=false
for attempt in 1 2 3 4 5; do
  if rosdep update --rosdistro jazzy; then
    rosdep_updated=true
    break
  fi
  echo "rosdep update attempt ${attempt} failed; retrying..." >&2
  sleep 2
done
if [[ "${rosdep_updated}" != "true" ]]; then
  echo "rosdep update failed after five attempts." >&2
  if [[ "${rosdep_local}" != "true" ]]; then
    echo "Run scripts/cache_rosdep_sources.ps1 on Windows and retry." >&2
  fi
  exit 4
fi

python3 -m venv --system-site-packages /opt/strawberry_venv
/opt/strawberry_venv/bin/python -m pip install --upgrade pip wheel

wheelhouse="${STRAWBERRY_WHEELHOUSE:-}"
if [[ -z "${wheelhouse}" ]]; then
  wheelhouse_candidates=(
    "${repo_root}/.cache/wheels"
    /opt/strawberry_wheelhouse
    /home/*/strawberry_wheels
  )
  for candidate in "${wheelhouse_candidates[@]}"; do
    if [[ -f "${candidate}/torch-2.13.0-cp312-cp312-manylinux_2_28_x86_64.whl" &&
          -f "${candidate}/ultralytics-8.4.92-py3-none-any.whl" &&
          -f "${candidate}/setuptools-79.0.1-py3-none-any.whl" ]]; then
      wheelhouse="${candidate}"
      break
    fi
  done
fi

if [[ -n "${wheelhouse}" ]]; then
  /opt/strawberry_venv/bin/pip install \
    --no-index \
    --find-links "${wheelhouse}" \
    -r "${repo_root}/requirements/perception.txt"
else
  /opt/strawberry_venv/bin/pip install -r "${repo_root}/requirements/perception.txt"
fi

/opt/strawberry_venv/bin/python - <<'PY'
import torch
import ultralytics

print(f"ultralytics={ultralytics.__version__}")
print(f"torch={torch.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
if not torch.cuda.is_available():
    raise SystemExit("CUDA is not visible inside the WSL Python environment")
PY

cat >/etc/profile.d/strawberry-ros.sh <<'EOF'
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ -f /etc/ros/rosdep/local-sources/index-v4.yaml ]]; then
  export ROSDISTRO_INDEX_URL=file:///etc/ros/rosdep/local-sources/index-v4.yaml
fi
EOF

echo "ROS 2 Jazzy environment installed. Start a new WSL shell before building."
