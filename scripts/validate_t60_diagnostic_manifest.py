#!/usr/bin/env python3
"""Validate and optionally list the frozen T60 position diagnostic manifest."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.diagnostics import (  # noqa: E402
    load_oracle_shadow_diagnostic_manifest,
    verify_shadow_model,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--emit-tsv", action="store_true")
    parser.add_argument("--emit-park-tsv", action="store_true")
    arguments = parser.parse_args()
    manifest = load_oracle_shadow_diagnostic_manifest(arguments.manifest)
    verify_shadow_model(manifest, REPOSITORY_ROOT, arguments.model)
    if arguments.emit_tsv and arguments.emit_park_tsv:
        raise SystemExit("choose only one TSV output mode")
    if arguments.emit_tsv:
        for scenario in manifest.scenarios:
            xyz = scenario.target_position_m
            print(
                "\t".join(
                    (
                        scenario.scenario_id,
                        scenario.position_label,
                        scenario.target_model_name,
                        str(manifest.control_target_id),
                        *(format(value, ".12g") for value in xyz),
                    )
                )
            )
    elif arguments.emit_park_tsv:
        for model in manifest.parked_models:
            print(
                "\t".join(
                    (
                        model.model_name,
                        str(model.target_id),
                        *(format(value, ".12g") for value in model.position_m),
                    )
                )
            )
    else:
        print(
            f"validated {manifest.diagnostic_id}: {len(manifest.scenarios)} "
            "non-acceptance scenarios, oracle control, baseline__best shadow, "
            f"scene_mode={manifest.scene_mode}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
