#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
exec /opt/strawberry_venv/bin/python "${repo_root}/scripts/verify_environment.py"
