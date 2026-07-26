#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
models="${repo_root}/ros2_ws/src/strawberry_sim/models"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export GZ_SIM_RESOURCE_PATH="${models}:${GZ_SIM_RESOURCE_PATH:-}"

canonical_assets=(
  strawberry_ripe
  strawberry_unripe
  strawberry_plant_v2
)
for asset in "${canonical_assets[@]}"; do
  model_path="${models}/${asset}/model.sdf"
  gz sdf -k "${model_path}"
done

variants=(
  strawberry_ripe_textured
  strawberry_unripe_textured
  strawberry_ripe_realistic
  strawberry_unripe_realistic
  strawberry_ripe_textured_v2
  strawberry_unripe_textured_v2
  strawberry_ripe_realistic_v2
  strawberry_unripe_realistic_v2
  strawberry_ripe_realistic_v3
  strawberry_unripe_realistic_v3
  strawberry_ripe_realistic_v4
  strawberry_unripe_realistic_v4
)
for variant in "${variants[@]}"; do
  model_path="${models}/${variant}/model.sdf"
  texture_count="$(find "${models}/${variant}/materials/textures" -maxdepth 1 -type f -name '*.png' | wc -l)"
  if [[ "${texture_count}" -ne 1 ]]; then
    echo "${variant} must contain exactly one versioned PNG texture" >&2
    exit 2
  fi
  if [[ "${variant}" == *_v2 ]]; then
    grep -q '_v2.png' "${model_path}" || {
      echo "${variant} does not reference its active v2 texture" >&2
      exit 2
    }
  fi
  gz sdf -k "${model_path}"
done

for variant in \
  strawberry_ripe_realistic strawberry_unripe_realistic \
  strawberry_ripe_realistic_v2 strawberry_unripe_realistic_v2 \
  strawberry_ripe_realistic_v3 strawberry_unripe_realistic_v3; do
  mesh="${models}/${variant}/meshes/strawberry_body_v1.obj"
  [[ -s "${mesh}" ]] || { echo "missing mesh: ${mesh}" >&2; exit 3; }
done

echo "Validated ${#canonical_assets[@]} canonical models and ${#variants[@]} asset-ablation models."
