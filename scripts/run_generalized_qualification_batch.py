"""Execute and score the five preselected non-formal qualification scenes once."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_bringup"))

from strawberry_bringup.development_gate import summarize_runtime_gate  # noqa: E402
from strawberry_bringup.feasibility_sweep import (  # noqa: E402
    BASE_CAMERA_RESOLUTIONS,
    load_json,
    receipt_file,
    sha256_file,
)


def _git(command: Sequence[str]) -> str:
    return subprocess.run(
        ["git", *command],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _write_new_json(path: Path, payload: dict) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _verify_receipt(binding: dict, expected_path: Path) -> None:
    bound_path = (ROOT / str(binding.get("path", ""))).resolve()
    if bound_path != expected_path.resolve():
        raise ValueError(f"qualification binding points at another file: {expected_path}")
    if not expected_path.is_file():
        raise ValueError(f"qualification input is missing: {expected_path}")
    if int(binding.get("size_bytes", -1)) != expected_path.stat().st_size:
        raise ValueError(f"qualification input size changed: {expected_path}")
    if str(binding.get("sha256", "")) != sha256_file(expected_path):
        raise ValueError(f"qualification input hash changed: {expected_path}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep-summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    options = parser.parse_args(argv)
    output_dir = options.output_dir.resolve()
    try:
        output_dir.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise ValueError("output directory must stay inside the repository") from exc
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError("refusing to use a non-empty qualification directory")
    if _git(("status", "--porcelain")):
        raise RuntimeError("qualification behavior requires a clean frozen Git tree")

    sweep_path = options.sweep_summary.resolve()
    sweep = load_json(sweep_path)
    selected = sweep.get("selected_runtime_scenario_ids")
    if (
        sweep.get("scope") != "GENERALIZED_DEVELOPMENT_ZERO_MOTION_SWEEP"
        or sweep.get("split") != "qualification"
        or sweep.get("selection_complete") is not True
        or not isinstance(selected, list)
        or len(selected) != 5
        or len(set(selected)) != 5
    ):
        raise ValueError("qualification sweep did not freeze exactly five scenes")
    commit = _git(("rev-parse", "HEAD"))
    if sweep.get("git_commit") != commit:
        raise ValueError("qualification sweep Git binding no longer matches HEAD")
    if sweep.get("git_worktree_clean") is not True:
        raise ValueError("qualification sweep was not produced from a clean Git tree")
    base_camera_resolution = str(sweep.get("base_camera_resolution", ""))
    if base_camera_resolution not in BASE_CAMERA_RESOLUTIONS:
        raise ValueError("qualification sweep has an unsupported base-camera profile")
    scenario_rows = {
        str(row["scenario_id"]): row for row in sweep.get("scenarios", [])
    }
    if set(selected) - set(scenario_rows):
        raise ValueError("qualification selection references an absent scenario")
    scene_dir = sweep_path.parent / "scenes"
    try:
        scene_dir_relative = scene_dir.relative_to(ROOT)
    except ValueError as exc:
        raise ValueError("qualification scenes must stay inside the repository") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    runs_dir = output_dir / "runs"
    runs_dir.mkdir()
    run_rows = []
    scores = []
    for order, scenario_id in enumerate(selected, start=1):
        row = scenario_rows[scenario_id]
        if not (
            row.get("eligible") is True
            and row.get("moveit_eligible") is True
            and row.get("observability_eligible") is True
            and row.get("camera_resolution_matches_requested") is True
            and row.get("truth_isolation_pass") is True
            and row.get("cleanup_clean") is True
        ):
            raise ValueError(
                f"selected scene did not pass every qualification gate: {scenario_id}"
            )
        seed = int(row["seed"])
        scene_path = scene_dir / f"generalized_seed_{seed:06d}.yaml"
        world_path = scene_dir / f"generalized_seed_{seed:06d}.sdf"
        files = row.get("files")
        if not isinstance(files, dict):
            raise ValueError(
                f"selected scene has no immutable file bindings: {scenario_id}"
            )
        for name, expected_path in (("scene", scene_path), ("world", world_path)):
            binding = files.get(name)
            if not isinstance(binding, dict):
                raise ValueError(
                    f"selected scene has no {name} binding: {scenario_id}"
                )
            _verify_receipt(binding, expected_path)
        run_dir = runs_dir / scenario_id
        run_dir.mkdir()
        environment = os.environ.copy()
        environment["STRAWBERRY_DEVELOPMENT_OUTPUT_DIR"] = str(run_dir)
        environment["STRAWBERRY_BASE_CAMERA_RESOLUTION"] = base_camera_resolution
        with (run_dir / "runner.stdout.log").open(
            "x", encoding="utf-8", newline="\n"
        ) as stdout, (run_dir / "runner.stderr.log").open(
            "x", encoding="utf-8", newline="\n"
        ) as stderr:
            result = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/run_generalized_development_probe.sh"),
                    str(seed),
                    str(scene_dir_relative),
                    scenario_id,
                ],
                cwd=ROOT,
                env=environment,
                stdout=stdout,
                stderr=stderr,
                check=False,
            )
        score_path = run_dir / "runtime_score.json"
        score = load_json(score_path) if score_path.exists() else None
        if score is not None:
            scores.append(score)
        files = {}
        for name in (
            "runtime_probe.json",
            "runtime_score.json",
            "truth_isolation.json",
            "cleanup_probe.json",
            "launch.log",
            "runner.stdout.log",
            "runner.stderr.log",
        ):
            path = run_dir / name
            if path.exists():
                files[name] = receipt_file(path, ROOT)
        run_rows.append(
            {
                "order": order,
                "scenario_id": scenario_id,
                "seed": seed,
                "runner_exit_code": int(result.returncode),
                "score_available": score is not None,
                "run_safety_pass": bool(
                    score and score.get("run_safety_pass") is True
                ),
                "files": files,
            }
        )
        print(
            json.dumps(
                {
                    "order": order,
                    "scenario_id": scenario_id,
                    "exit_code": result.returncode,
                    "score_available": score is not None,
                },
                sort_keys=True,
            ),
            flush=True,
        )

    gate = summarize_runtime_gate(scores) if len(scores) == 5 else None
    runtime_bindings = {}
    for relative in (
        "ros2_ws/src/strawberry_bringup/launch/generalized_harvest.launch.py",
        "ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_orchestrator.py",
        "ros2_ws/src/strawberry_bringup/strawberry_bringup/target_selector.py",
        "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/action_server.py",
        "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/core.py",
        "config/generalized_runtime_development_matrix_v2.json",
    ):
        path = ROOT / relative
        runtime_bindings[relative] = sha256_file(path)
    summary = {
        "schema_version": 1,
        "scope": "GENERALIZED_DEVELOPMENT_QUALIFICATION_BATCH",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "git_commit": commit,
        "git_worktree_clean": True,
        "sweep_summary": receipt_file(sweep_path, ROOT),
        "base_camera_resolution": base_camera_resolution,
        "model_sha256": sweep.get("model_sha256"),
        "runtime_bindings": runtime_bindings,
        "selected_runtime_scenario_ids": selected,
        "runs": run_rows,
        "gate": gate,
        "overall_pass": bool(gate and gate.get("overall_pass") is True),
    }
    summary_path = output_dir / "qualification_runtime_summary.json"
    _write_new_json(summary_path, summary)
    print(json.dumps({"summary": str(summary_path), "overall_pass": summary["overall_pass"]}))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
