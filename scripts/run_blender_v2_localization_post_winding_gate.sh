#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${repo_root}/scripts/run_blender_v2_localization_gate.sh" \
  "${repo_root}/config/blender_v2_localization_accuracy_100_post_winding_v1.json"
