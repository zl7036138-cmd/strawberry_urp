"""Run one immutable 18-seed generalized zero-motion feasibility sweep."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "strawberry_bringup"))

from strawberry_bringup.feasibility_sweep import (  # noqa: E402
    BASE_CAMERA_RESOLUTIONS,
    load_json,
    observability_receipt_is_eligible,
    observability_resolution_matches,
    parse_base_camera_resolution,
    receipt_file,
    scenario_rows,
    select_runtime_scenarios,
    sha256_file,
    validate_development_matrix,
)


def run_logged(command, *, stdout_path: Path, stderr_path: Path, env=None) -> int:
    with stdout_path.open("x", encoding="utf-8", newline="\n") as stdout, stderr_path.open(
        "x", encoding="utf-8", newline="\n"
    ) as stderr:
        result = subprocess.run(
            command,
            cwd=ROOT,
            env=env,
            stdout=stdout,
            stderr=stderr,
            check=False,
        )
    return int(result.returncode)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--matrix",
        type=Path,
        default=ROOT / "config" / "generalized_runtime_development_matrix_v2.json",
    )
    parser.add_argument(
        "--formal-matrix",
        type=Path,
        default=ROOT / "config" / "generalized_harvest_matrix_v1.json",
    )
    parser.add_argument("--split", choices=("discovery", "qualification"), required=True)
    parser.add_argument("--batch-index", type=int, default=0)
    parser.add_argument(
        "--base-camera-resolution",
        choices=tuple(BASE_CAMERA_RESOLUTIONS),
        default="320x240",
        help="One fixed overview-camera profile for the complete sweep.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    options = parser.parse_args(argv)
    base_camera_width, base_camera_height = parse_base_camera_resolution(
        options.base_camera_resolution
    )
    output_dir = options.output_dir.resolve()
    try:
        output_dir.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise ValueError("output directory must stay inside the repository") from exc
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError("refusing to use a non-empty feasibility sweep directory")
    git_status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if options.split == "qualification" and git_status:
        raise RuntimeError(
            "qualification screening requires a clean frozen Git tree"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    scenes_dir = output_dir / "scenes"
    runs_dir = output_dir / "runs"
    scenes_dir.mkdir()
    runs_dir.mkdir()

    config = load_json(options.matrix)
    formal = load_json(options.formal_matrix)
    formal_seeds = [int(row["seed"]) for row in formal["scenarios"]]
    validate_development_matrix(config, formal_seeds=formal_seeds)
    requires_observability_gate = int(config["schema_version"]) >= 2
    rows = scenario_rows(config, split=options.split, batch_index=options.batch_index)
    receipts = {}
    observability_receipts = {}
    audit_rows = []
    for order, row in enumerate(rows, start=1):
        scenario_id = str(row["scenario_id"])
        seed = int(row["seed"])
        scene_stdout = output_dir / f"{scenario_id}.materialize.stdout.log"
        scene_stderr = output_dir / f"{scenario_id}.materialize.stderr.log"
        materialize_status = run_logged(
            [
                "ros2",
                "run",
                "strawberry_sim",
                "generate_generalized_scene",
                "--base-scene",
                str(ROOT / "ros2_ws/src/strawberry_sim/config/scene.yaml"),
                "--base-world",
                str(ROOT / "ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"),
                "--output-dir",
                str(scenes_dir),
                "--seed",
                str(seed),
                "--profile",
                str(row["profile"]),
                "--plant-count",
                str(row["plant_count"]),
                "--position-band",
                str(row["position_band"]),
                "--occlusion",
                str(row["occlusion"]),
                "--layout-contract",
                str(row["layout_contract"]),
            ],
            stdout_path=scene_stdout,
            stderr_path=scene_stderr,
        )
        run_dir = runs_dir / scenario_id
        run_dir.mkdir()
        probe_status = None
        if materialize_status == 0:
            environment = os.environ.copy()
            environment["STRAWBERRY_FEASIBILITY_OUTPUT_DIR"] = str(run_dir)
            environment["STRAWBERRY_BASE_CAMERA_RESOLUTION"] = (
                options.base_camera_resolution
            )
            probe_status = run_logged(
                [
                    "bash",
                    str(ROOT / "scripts/run_generalized_feasibility_probe.sh"),
                    str(seed),
                    str(scenes_dir.relative_to(ROOT)),
                    scenario_id,
                ],
                stdout_path=run_dir / "runner.stdout.log",
                stderr_path=run_dir / "runner.stderr.log",
                env=environment,
            )
        probe_path = run_dir / "feasibility_probe.json"
        observability_path = run_dir / "observability.json"
        cleanup_path = run_dir / "cleanup_probe.json"
        truth_path = run_dir / "truth_isolation.json"
        probe = load_json(probe_path) if probe_path.exists() else None
        observability = (
            load_json(observability_path) if observability_path.exists() else None
        )
        cleanup = load_json(cleanup_path) if cleanup_path.exists() else None
        truth = load_json(truth_path) if truth_path.exists() else None
        scene_path = scenes_dir / f"generalized_seed_{seed:06d}.yaml"
        world_path = scenes_dir / f"generalized_seed_{seed:06d}.sdf"
        if probe is not None:
            receipts[scenario_id] = probe
        if observability is not None:
            observability_receipts[scenario_id] = observability
        moveit_eligible = bool(
            probe and probe.get("eligible_for_multi_fruit_runtime") is True
        )
        resolution_matches = observability_resolution_matches(
            observability, options.base_camera_resolution
        )
        observability_eligible = (
            resolution_matches
            and observability_receipt_is_eligible(observability)
        )
        audit_rows.append(
            {
                "order": order,
                **dict(row),
                "materialize_exit_code": materialize_status,
                "probe_exit_code": probe_status,
                "moveit_eligible": moveit_eligible,
                "camera_resolution_matches_requested": resolution_matches,
                "observability_eligible": observability_eligible,
                "eligible": moveit_eligible
                and (
                    observability_eligible
                    if requires_observability_gate
                    else True
                ),
                "cleanup_clean": bool(cleanup and cleanup.get("outcome") == "CLEAN"),
                "truth_isolation_pass": bool(truth and truth.get("overall_pass") is True),
                "files": {
                    name: receipt_file(path, ROOT)
                    for name, path in (
                        ("scene", scene_path),
                        ("world", world_path),
                        ("probe", probe_path),
                        ("observability", observability_path),
                        ("cleanup", cleanup_path),
                        ("truth_isolation", truth_path),
                    )
                    if path.exists()
                },
            }
        )
        print(
            json.dumps(
                {
                    "order": order,
                    "scenario_id": scenario_id,
                    "eligible": audit_rows[-1]["eligible"],
                    "probe_exit_code": probe_status,
                },
                sort_keys=True,
            ),
            flush=True,
        )
    selected = select_runtime_scenarios(
        rows,
        receipts,
        observability_receipts=(
            {
                scenario_id: receipt
                for scenario_id, receipt in observability_receipts.items()
                if observability_resolution_matches(
                    receipt, options.base_camera_resolution
                )
            }
            if requires_observability_gate
            else None
        ),
        count=int(config["qualification"]["selected_runtime_scenarios"]),
    )
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    model = ROOT / "outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt"
    summary = {
        "schema_version": 1,
        "scope": "GENERALIZED_DEVELOPMENT_ZERO_MOTION_SWEEP",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "split": options.split,
        "batch_index": options.batch_index,
        "matrix_sha256": sha256_file(options.matrix),
        "formal_matrix_sha256": sha256_file(options.formal_matrix),
        "git_commit": commit,
        "git_worktree_clean": not bool(git_status),
        "model_sha256": sha256_file(model),
        "localization_config_sha256": sha256_file(
            ROOT
            / "ros2_ws/src/strawberry_localization/config/localization_generalized.yaml"
        ),
        "base_camera_resolution": options.base_camera_resolution,
        "base_camera_image_shape_hw": [
            base_camera_height,
            base_camera_width,
        ],
        "scenario_count": len(rows),
        "moveit_eligible_count": sum(row["moveit_eligible"] for row in audit_rows),
        "observability_eligible_count": sum(
            row["observability_eligible"] for row in audit_rows
        ),
        "eligible_count": sum(row["eligible"] for row in audit_rows),
        "selected_runtime_scenario_ids": list(selected),
        "selection_complete": len(selected) == 5,
        "scenarios": audit_rows,
    }
    summary_path = output_dir / "sweep_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"summary": str(summary_path), "selection_complete": summary["selection_complete"]}))
    return 0 if summary["selection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
