#!/usr/bin/env python3
"""Generate the P5 release tables and figures from immutable gate evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPOSITORY_ROOT / "artifacts" / "p5" / "release_v1"

INPUT_PATHS = {
    "t30": "artifacts/perception/t30_train_audit_outcome_handoff_v1.json",
    "t40": "results/p2/localization_gate_v6/summary.json",
    "t50": "results/p2/oracle_gate/summary.json",
    "t60": "results/t60/oracle_gate_v2/summary.json",
    "p3_development": "results/p3/perception_repeated_dev_gate_v1/summary.json",
    "p3_formal": "results/p3/formal_matrix_v1/summary.json",
    "p3_handoff": "artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json",
    "p4_qualification": "results/p4/sim_adapt_qualification_v1/summary.json",
    "p4_handoff": "artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json",
    "p5_smoke_reference": "results/p5/release_smoke_reference_v1/summary.json",
    "p5_clean_build": "results/p5/clean_build_v1/summary.json",
    "p5_smoke_clean": "results/p5/release_smoke_clean_v1/summary.json",
    "p5_clean_handoff": "artifacts/p5/p5_clean_reproduction_handoff_v1.json",
    "p5_video_receipt": "artifacts/p5/video/strawberry_urp_demo_v1.receipt.json",
    "p5_video": "artifacts/p5/video/strawberry_urp_demo_v1.mp4",
    "p3_decision": "docs/decisions/0030-record-formal-p3-simulator-matrix-outcome.md",
    "p4_decision": "docs/decisions/0032-reject-single-p4-simulator-perception-intervention.md",
    "p5_clean_decision": "docs/decisions/0033-accept-clean-p5-release-reproduction.md",
    "p5_release_decision": "docs/decisions/0034-freeze-final-p5-release.md",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _binding(path: Path) -> dict[str, Any]:
    return {
        "path": path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _outcome(passed: bool) -> str:
    return "PASS" if passed else "FAIL"


def _percent(value: float) -> str:
    return f"{100.0 * value:.2f}%"


def _metric_rows(data: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    t30 = data["t30"]
    t40 = data["t40"]
    t50 = data["t50"]
    t60 = data["t60"]
    dev = data["p3_development"]
    p3 = data["p3_formal"]
    p4 = data["p4_qualification"]
    p5_reference = data["p5_smoke_reference"]
    p5_clean_build = data["p5_clean_build"]
    p5_clean = data["p5_smoke_clean"]
    p5_video = data["p5_video_receipt"]

    rows = [
        {
            "stage": "P2/T30",
            "metric": "real_validation_macro_f1",
            "value": t30["validation"]["macro_f1"],
            "requirement": f">={t30['validation']['macro_f1_min']}",
            "outcome": _outcome(
                t30["validation"]["macro_f1"]
                >= t30["validation"]["macro_f1_min"]
            ),
            "evidence": INPUT_PATHS["t30"],
            "scope": "audited real validation; held-out test sealed",
        },
        {
            "stage": "P2/T40",
            "metric": "localization_median_error_mm",
            "value": t40["summary"]["median_error_mm"],
            "requirement": f"<={t40['summary']['median_limit_mm']}",
            "outcome": _outcome(t40["summary"]["passed"]),
            "evidence": INPUT_PATHS["t40"],
            "scope": "100 simulator truth positions",
        },
        {
            "stage": "P2/T40",
            "metric": "localization_p95_error_mm",
            "value": t40["summary"]["p95_error_mm"],
            "requirement": f"<={t40['summary']['p95_limit_mm']}",
            "outcome": _outcome(t40["summary"]["passed"]),
            "evidence": INPUT_PATHS["t40"],
            "scope": "100 simulator truth positions",
        },
        {
            "stage": "P2/T50",
            "metric": "truth_target_pick_success_rate",
            "value": t50["success_rate"],
            "requirement": f">={t50['required_success_rate']}",
            "outcome": _outcome(t50["gate_passed"]),
            "evidence": INPUT_PATHS["t50"],
            "scope": f"{t50['trial_count']} fresh simulator worlds",
        },
        {
            "stage": "P3/T60",
            "metric": "oracle_integration_success_rate",
            "value": t60["success_rate"],
            "requirement": f">={t60['required_success_rate']}",
            "outcome": _outcome(t60["gate_passed"]),
            "evidence": INPUT_PATHS["t60"],
            "scope": f"{t60['trial_count']} fresh simulator worlds",
        },
        {
            "stage": "P3/development",
            "metric": "clear_scene_perception_control_success_rate",
            "value": dev["positive_success_rate"],
            "requirement": f">={dev['minimum_positive_success_rate']}",
            "outcome": _outcome(dev["development_gate_passed"]),
            "evidence": INPUT_PATHS["p3_development"],
            "scope": "non-formal, one clear pose, engineering waiver",
        },
        {
            "stage": "P3/formal",
            "metric": "positive_end_to_end_success_rate",
            "value": p3["metrics"]["positive_success_rate"],
            "requirement": f">={p3['metrics']['minimum_positive_success_rate']}",
            "outcome": _outcome(p3["metrics"]["formal_p3_simulator_gate_passed"]),
            "evidence": INPUT_PATHS["p3_formal"],
            "scope": "135 fixed simulator positives",
        },
        {
            "stage": "P3/formal",
            "metric": "only_unripe_safe_no_pick_rate",
            "value": p3["metrics"]["negative_safe_no_pick_rate"],
            "requirement": f">={p3['metrics']['minimum_negative_safe_no_pick_rate']}",
            "outcome": _outcome(
                p3["metrics"]["negative_safe_no_pick_rate"]
                >= p3["metrics"]["minimum_negative_safe_no_pick_rate"]
            ),
            "evidence": INPUT_PATHS["p3_formal"],
            "scope": "30 fixed simulator negatives",
        },
        {
            "stage": "P3/formal",
            "metric": "planning_time_p95_sec",
            "value": p3["metrics"]["planning_time_p95_sec"],
            "requirement": f"<={p3['metrics']['maximum_planning_time_p95_sec']}",
            "outcome": _outcome(
                p3["metrics"]["planning_time_p95_sec"]
                <= p3["metrics"]["maximum_planning_time_p95_sec"]
            ),
            "evidence": INPUT_PATHS["p3_formal"],
            "scope": "51 formal planning observations",
        },
        {
            "stage": "P4/qualification",
            "metric": "heavy_occlusion_ripe_detection_frame_rate",
            "value": p4["metrics"]["heavy_overall"]["ripe_frame_rate"],
            "requirement": ">=0.95",
            "outcome": _outcome(p4["metrics"]["checks"]["heavy_ripe_overall"]),
            "evidence": INPUT_PATHS["p4_qualification"],
            "scope": "300 no-motion simulator frames",
        },
        {
            "stage": "P4/qualification",
            "metric": "heavy_occlusion_target_pose_frame_rate",
            "value": p4["metrics"]["heavy_overall"]["target_pose_frame_rate"],
            "requirement": ">=0.90",
            "outcome": _outcome(p4["metrics"]["checks"]["heavy_target_overall"]),
            "evidence": INPUT_PATHS["p4_qualification"],
            "scope": "300 no-motion simulator frames",
        },
        {
            "stage": "P4/qualification",
            "metric": "unripe_false_ripe_frame_rate",
            "value": p4["metrics"]["unripe"]["false_ripe_frame_rate"],
            "requirement": "<=0.01",
            "outcome": _outcome(p4["metrics"]["checks"]["unripe_false_ripe_rate"]),
            "evidence": INPUT_PATHS["p4_qualification"],
            "scope": "900 no-motion simulator frames",
        },
        {
            "stage": "P5/reference",
            "metric": "release_smoke_behavior_pass_rate",
            "value": p5_reference["behavior_pass_rate"],
            "requirement": "reference",
            "outcome": _outcome(p5_reference["smoke_passed"]),
            "evidence": INPUT_PATHS["p5_smoke_reference"],
            "scope": "5 clear positives + 5 clear only-unripe negatives; current environment",
        },
        {
            "stage": "P5/clean-build",
            "metric": "colcon_tests_passed",
            "value": p5_clean_build["tests"]["tests"],
            "requirement": "246 tests; 0 errors/failures/skips",
            "outcome": p5_clean_build["status"],
            "evidence": INPUT_PATHS["p5_clean_build"],
            "scope": "separate Ubuntu 24.04 WSL2 distribution; seven packages",
        },
        {
            "stage": "P5/clean-smoke",
            "metric": "release_smoke_behavior_pass_rate",
            "value": p5_clean["behavior_pass_rate"],
            "requirement": "reference absolute difference <=0.05",
            "outcome": _outcome(p5_clean["clean_reproduction_passed"]),
            "evidence": INPUT_PATHS["p5_smoke_clean"],
            "scope": "5 clear positives + 5 clear only-unripe negatives; clean environment",
        },
        {
            "stage": "P5/video",
            "metric": "final_video_duration_sec",
            "value": p5_video["video"]["duration_sec"],
            "requirement": "240-360 sec; H.264 1280x720",
            "outcome": p5_video["status"],
            "evidence": INPUT_PATHS["p5_video_receipt"],
            "scope": "simulation-only; live segments are explicitly non-formal",
        },
    ]
    return rows


def _bar_chart(
    title: str,
    labels: list[str],
    values: list[float],
    *,
    target: float | None = None,
    color: str = "#2f6f9f",
) -> str:
    width, height = 900, 480
    left, right, top, bottom = 210, 45, 70, 70
    plot_width = width - left - right
    plot_height = height - top - bottom
    count = len(labels)
    slot = plot_height / count
    bar_height = min(38.0, slot * 0.58)
    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="480" viewBox="0 0 900 480">',
        '<rect width="900" height="480" fill="white"/>',
        f'<text x="450" y="35" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="700">{html.escape(title)}</text>',
    ]
    for tick in range(0, 6):
        value = tick / 5
        x = left + value * plot_width
        lines.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_height}" stroke="#e3e8ed" stroke-width="1"/>'
        )
        lines.append(
            f'<text x="{x:.1f}" y="{height - 38}" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#4b5563">{int(value * 100)}%</text>'
        )
    if target is not None:
        x = left + target * plot_width
        lines.append(
            f'<line x1="{x:.1f}" y1="{top - 8}" x2="{x:.1f}" y2="{top + plot_height}" stroke="#c62828" stroke-width="3" stroke-dasharray="8 5"/>'
        )
        lines.append(
            f'<text x="{x - 6:.1f}" y="{height - 12}" text-anchor="end" font-family="sans-serif" font-size="13" fill="#c62828">gate {target:.0%}</text>'
        )
    for index, (label, value) in enumerate(zip(labels, values)):
        y = top + index * slot + (slot - bar_height) / 2
        lines.append(
            f'<text x="{left - 14}" y="{y + bar_height / 2 + 5:.1f}" text-anchor="end" font-family="sans-serif" font-size="15" fill="#1f2937">{html.escape(label)}</text>'
        )
        lines.append(
            f'<rect x="{left}" y="{y:.1f}" width="{max(0.0, min(1.0, value)) * plot_width:.1f}" height="{bar_height:.1f}" rx="3" fill="{color}"/>'
        )
        lines.append(
            f'<text x="{left + max(0.0, min(1.0, value)) * plot_width + 8:.1f}" y="{y + bar_height / 2 + 5:.1f}" font-family="sans-serif" font-size="15" font-weight="700" fill="#111827">{value:.1%}</text>'
        )
    lines.append("</svg>\n")
    return "\n".join(lines)


def _grouped_chart(title: str, groups: dict[str, tuple[float, float]]) -> str:
    width, height = 900, 500
    left, right, top, bottom = 100, 45, 75, 105
    plot_width = width - left - right
    plot_height = height - top - bottom
    labels = list(groups)
    slot = plot_width / len(labels)
    bar_width = min(50.0, slot * 0.30)
    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500" viewBox="0 0 900 500">',
        '<rect width="900" height="500" fill="white"/>',
        f'<text x="450" y="35" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="700">{html.escape(title)}</text>',
    ]
    for tick in range(0, 6):
        value = tick / 5
        y = top + (1 - value) * plot_height
        lines.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="#e3e8ed"/>'
        )
        lines.append(
            f'<text x="{left - 12}" y="{y + 5:.1f}" text-anchor="end" font-family="sans-serif" font-size="14" fill="#4b5563">{int(value * 100)}%</text>'
        )
    for index, label in enumerate(labels):
        detection, target_pose = groups[label]
        center = left + (index + 0.5) * slot
        for offset, value, color in (
            (-bar_width / 2, detection, "#2f6f9f"),
            (bar_width / 2, target_pose, "#e38832"),
        ):
            x = center + offset - bar_width / 2
            bar_height = value * plot_height
            y = top + plot_height - bar_height
            lines.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width:.1f}" height="{bar_height:.1f}" rx="3" fill="{color}"/>'
            )
            lines.append(
                f'<text x="{x + bar_width / 2:.1f}" y="{max(top + 14, y - 7):.1f}" text-anchor="middle" font-family="sans-serif" font-size="13" font-weight="700">{value:.1%}</text>'
            )
        lines.append(
            f'<text x="{center:.1f}" y="{top + plot_height + 28}" text-anchor="middle" font-family="sans-serif" font-size="14">{html.escape(label)}</text>'
        )
    lines.extend(
        [
            '<rect x="285" y="455" width="18" height="18" fill="#2f6f9f"/>',
            '<text x="310" y="469" font-family="sans-serif" font-size="14">RIPE detection</text>',
            '<rect x="475" y="455" width="18" height="18" fill="#e38832"/>',
            '<text x="500" y="469" font-family="sans-serif" font-size="14">target pose</text>',
            "</svg>\n",
        ]
    )
    return "\n".join(lines)


def _summary_markdown(metrics: dict[str, Any]) -> str:
    p3 = metrics["formal_p3"]
    p4 = metrics["p4_qualification"]
    return f"""# P5 发布摘要 v1

本包由冻结的 P2、P3 和 P4 JSON 证据自动生成。正式测试集仍封存，本文不包含实体机器人或 sim-to-real 结论。

## 结论

- T40 三维定位通过：中位误差 {metrics['module_results']['t40_localization']['median_error_mm']:.3f} mm，95 分位误差 {metrics['module_results']['t40_localization']['p95_error_mm']:.3f} mm。
- T50 真值目标抓取通过：9/10，成功率 90%。
- T60 Oracle 集成通过：10/10；单一清晰位姿的感知控制开发门也通过 10/10 正例与 10/10 负例，但它不是正式 P3 证据。
- T30 原始数值门未通过：真实验证宏平均 F1 为 {metrics['module_results']['t30_perception']['macro_f1']:.6f}，低于 0.85；模型仅凭 ADR 0026 工程豁免用于仿真。
- 正式 P3 未通过：{p3['positive_successes']}/{p3['positive_trials']}，即 {_percent(p3['positive_success_rate'])}，低于 80% 门槛；30/30 仅未成熟场景安全返回 `NO_PICK`。
- P4 唯一干预未通过资格测试：重遮挡成熟检测达到 {_percent(p4['heavy_ripe_detection_rate'])}，但目标位姿发布率为 {_percent(p4['heavy_target_pose_rate'])}，因此未授权运动复测。
- P5 当前环境参考冒烟通过：{metrics['release_reproduction']['reference_behavior_passes']}/{metrics['release_reproduction']['reference_trials']}。
- P5 独立干净 WSL2 复现通过：7 个包、{metrics['release_reproduction']['clean_build_tests']} 项测试全绿，干净环境冒烟 {metrics['release_reproduction']['clean_behavior_passes']}/{metrics['release_reproduction']['clean_trials']}，与参考行为通过率绝对差 {metrics['release_reproduction']['reference_absolute_difference']:.3f}。
- P5 最终演示视频通过：{metrics['release_video']['duration_sec']:.1f} 秒，H.264，1280×720；成功抓放明确标注为 Oracle 控制的非正式演示，仅未成熟片段安全返回 `NO_PICK`。

## 当前发布状态

`FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4`：P5 复现、报告、图表和视频交付均已完成并冻结；正式 P3 与 P4 的失败结论保持不变。
"""


def generate(output_dir: Path) -> dict[str, Any]:
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    inputs: dict[str, dict[str, Any]] = {}
    bindings: list[dict[str, Any]] = []
    for key, relative_path in INPUT_PATHS.items():
        path = REPOSITORY_ROOT / relative_path
        if not path.is_file():
            raise ValueError(f"required evidence is missing: {relative_path}")
        bindings.append(_binding(path))
        if path.suffix == ".json":
            inputs[key] = _read_json(path)

    if inputs["p3_formal"]["held_out_test_receipt_exists"]:
        raise ValueError("held-out test receipt unexpectedly exists")
    if inputs["p3_formal"]["held_out_real_test_consumed"]:
        raise ValueError("held-out real test unexpectedly marked consumed")
    if not inputs["p3_formal"]["formal_matrix_consumed"]:
        raise ValueError("formal P3 matrix must be consumed before release reporting")
    if inputs["p3_formal"]["metrics"]["scenario_count"] != 165:
        raise ValueError("formal P3 summary does not contain 165 scenarios")
    if inputs["p4_qualification"]["metrics"]["scenario_count"] != 30:
        raise ValueError("P4 qualification summary does not contain 30 scenarios")

    t30 = inputs["t30"]
    t40 = inputs["t40"]
    t50 = inputs["t50"]
    t60 = inputs["t60"]
    dev = inputs["p3_development"]
    p3 = inputs["p3_formal"]
    p4 = inputs["p4_qualification"]
    p5_reference = inputs["p5_smoke_reference"]
    p5_clean_build = inputs["p5_clean_build"]
    p5_clean = inputs["p5_smoke_clean"]
    p5_clean_handoff = inputs["p5_clean_handoff"]
    p5_video = inputs["p5_video_receipt"]
    if p5_clean_build["status"] != "PASS":
        raise ValueError("clean P5 build receipt did not pass")
    if not p5_clean["clean_reproduction_passed"]:
        raise ValueError("clean P5 release smoke did not pass")
    if (
        p5_clean_handoff["status"]
        != "CLEAN_REPRODUCTION_ACCEPTED_VIDEO_PENDING"
    ):
        raise ValueError("clean P5 reproduction handoff is not accepted")
    if p5_video["status"] != "PASS":
        raise ValueError("P5 demonstration video did not pass")
    video_path = REPOSITORY_ROOT / p5_video["video"]["path"]
    if _sha256(video_path) != p5_video["video"]["sha256"]:
        raise ValueError("P5 demonstration video hash differs from its receipt")
    metrics = {
        "schema_version": 1,
        "kind": "p5_release_metrics",
        "release_id": "p5_release_v1",
        "status": "FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4",
        "stage_status": {
            "P0": "PASS",
            "P1": "PASS",
            "P2": "ENGINEERING_WAIVER_NUMERIC_GATE_FAILED",
            "P3": "FAIL",
            "P4": "FAIL",
            "P5": "PASS",
        },
        "module_results": {
            "t30_perception": {
                "macro_f1": t30["validation"]["macro_f1"],
                "ripe_f1": t30["validation"]["ripe_f1"],
                "unripe_f1": t30["validation"]["unripe_f1"],
                "macro_f1_gate": t30["validation"]["macro_f1_min"],
                "numeric_gate_passed": False,
                "engineering_waiver": True,
            },
            "t40_localization": {
                "samples": t40["summary"]["valid_measurements"],
                "median_error_mm": t40["summary"]["median_error_mm"],
                "p95_error_mm": t40["summary"]["p95_error_mm"],
                "passed": t40["summary"]["passed"],
            },
            "t50_truth_target_manipulation": {
                "successes": t50["successes"],
                "trials": t50["trial_count"],
                "success_rate": t50["success_rate"],
                "passed": t50["gate_passed"],
            },
            "t60_oracle_integration": {
                "successes": t60["successes"],
                "trials": t60["trial_count"],
                "success_rate": t60["success_rate"],
                "planning_time_p95_sec": t60["planning_time_p95_sec"],
                "passed": t60["gate_passed"],
            },
            "perception_control_development": {
                "positive_successes": dev["positive_successes"],
                "positive_trials": dev["positive_trials"],
                "negative_no_picks": dev["negative_no_picks"],
                "negative_trials": dev["negative_trials"],
                "passed": dev["development_gate_passed"],
                "formal_evidence": False,
            },
        },
        "formal_p3": {
            "positive_successes": p3["metrics"]["positive_successes"],
            "positive_trials": p3["metrics"]["positive_trials"],
            "positive_success_rate": p3["metrics"]["positive_success_rate"],
            "required_success_rate": p3["metrics"]["minimum_positive_success_rate"],
            "negative_safe_no_picks": p3["metrics"]["negative_safe_no_picks"],
            "negative_trials": p3["metrics"]["negative_trials"],
            "negative_false_pick_rate": p3["metrics"]["negative_fruit_picked_rate"],
            "planning_time_p95_sec": p3["metrics"]["planning_time_p95_sec"],
            "failure_by_category": p3["metrics"]["failure_by_category"],
            "positive_subgroups": p3["metrics"]["positive_subgroups"],
            "passed": p3["metrics"]["formal_p3_simulator_gate_passed"],
        },
        "p4_qualification": {
            "scenarios": p4["metrics"]["scenario_count"],
            "heavy_ripe_detection_rate": p4["metrics"]["heavy_overall"]["ripe_frame_rate"],
            "heavy_target_pose_rate": p4["metrics"]["heavy_overall"]["target_pose_frame_rate"],
            "unripe_correct_rate": p4["metrics"]["unripe"]["correct_unripe_frame_rate"],
            "unripe_false_ripe_rate": p4["metrics"]["unripe"]["false_ripe_frame_rate"],
            "passed": p4["metrics"]["qualification_passed"],
            "intervention_promoted": False,
        },
        "release_reproduction": {
            "reference_trials": p5_reference["trial_count"],
            "reference_behavior_passes": p5_reference["behavior_passes"],
            "reference_behavior_pass_rate": p5_reference["behavior_pass_rate"],
            "reference_positive_success_rate": p5_reference["positive_success_rate"],
            "reference_negative_no_pick_rate": p5_reference["negative_no_pick_rate"],
            "reference_planning_time_p95_sec": p5_reference["planning_time_p95_sec"],
            "reference_smoke_passed": p5_reference["smoke_passed"],
            "clean_distribution": p5_clean_build["environment"]["wsl_distribution"],
            "clean_build_packages": p5_clean_build["build"]["package_count"],
            "clean_build_tests": p5_clean_build["tests"]["tests"],
            "clean_build_errors": p5_clean_build["tests"]["errors"],
            "clean_build_failures": p5_clean_build["tests"]["failures"],
            "clean_build_skips": p5_clean_build["tests"]["skipped"],
            "clean_build_passed": p5_clean_build["status"] == "PASS",
            "clean_trials": p5_clean["trial_count"],
            "clean_behavior_passes": p5_clean["behavior_passes"],
            "clean_behavior_pass_rate": p5_clean["behavior_pass_rate"],
            "clean_positive_success_rate": p5_clean["positive_success_rate"],
            "clean_negative_no_pick_rate": p5_clean["negative_no_pick_rate"],
            "clean_planning_time_p95_sec": p5_clean["planning_time_p95_sec"],
            "reference_absolute_difference": p5_clean["reference_comparison"][
                "absolute_difference"
            ],
            "maximum_allowed_difference": p5_clean["reference_comparison"][
                "maximum_difference"
            ],
            "clean_environment_smoke_completed": True,
            "clean_reproduction_passed": p5_clean["clean_reproduction_passed"],
        },
        "release_video": {
            "path": p5_video["video"]["path"],
            "sha256": p5_video["video"]["sha256"],
            "size_bytes": p5_video["video"]["size_bytes"],
            "duration_sec": p5_video["video"]["duration_sec"],
            "codec": p5_video["video"]["codec"],
            "width": p5_video["video"]["width"],
            "height": p5_video["video"]["height"],
            "passed": True,
            "formal_evidence": False,
        },
        "safety_and_scope": {
            "held_out_real_test_consumed": False,
            "held_out_test_receipt_exists": False,
            "physical_robot_evidence": False,
            "sim_to_real_claim": False,
            "formal_p3_rerun_authorized": False,
            "post_intervention_motion_matrix_authorized": False,
        },
        "source_bindings": bindings,
    }
    metrics_path = output_dir / "final_metrics_v1.json"
    _write_json(metrics_path, metrics)

    rows = _metric_rows(inputs)
    csv_path = output_dir / "final_metrics_v1.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "stage",
                "metric",
                "value",
                "requirement",
                "outcome",
                "scope",
                "evidence",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)

    subgroups = p3["metrics"]["positive_subgroups"]
    p3_chart = _bar_chart(
        "Formal P3 success rate by occlusion",
        ["none", "partial", "heavy"],
        [subgroups["occlusion"][key]["success_rate"] for key in ("none", "partial", "heavy")],
        target=p3["metrics"]["minimum_positive_success_rate"],
    )
    (output_dir / "p3_success_by_occlusion.svg").write_text(p3_chart, encoding="utf-8")
    position_order = ("center", "far_left", "far_right", "near_left", "near_right")
    position_chart = _bar_chart(
        "Formal P3 success rate by position",
        list(position_order),
        [subgroups["position"][key]["success_rate"] for key in position_order],
        target=p3["metrics"]["minimum_positive_success_rate"],
        color="#497d46",
    )
    (output_dir / "p3_success_by_position.svg").write_text(position_chart, encoding="utf-8")
    p4_groups = {
        "nominal / none": (
            p4["metrics"]["no_occlusion"]["nominal_none"]["ripe_frame_rate"],
            p4["metrics"]["no_occlusion"]["nominal_none"]["target_pose_frame_rate"],
        ),
        "dim / none": (
            p4["metrics"]["no_occlusion"]["dim_none"]["ripe_frame_rate"],
            p4["metrics"]["no_occlusion"]["dim_none"]["target_pose_frame_rate"],
        ),
        "nominal / heavy": (
            p4["metrics"]["heavy_overall"]["ripe_frame_rate"],
            p4["metrics"]["heavy_overall"]["target_pose_frame_rate"],
        ),
    }
    (output_dir / "p4_detection_vs_target_pose.svg").write_text(
        _grouped_chart("P4 intervention: detection versus localizable target", p4_groups),
        encoding="utf-8",
    )
    (output_dir / "release_summary_v1.md").write_text(
        _summary_markdown(metrics), encoding="utf-8"
    )

    output_names = (
        "final_metrics_v1.json",
        "final_metrics_v1.csv",
        "p3_success_by_occlusion.svg",
        "p3_success_by_position.svg",
        "p4_detection_vs_target_pose.svg",
        "release_summary_v1.md",
    )
    manifest = {
        "schema_version": 1,
        "kind": "p5_release_evidence_manifest",
        "release_id": "p5_release_v1",
        "status": "FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4",
        "inputs": bindings,
        "outputs": [_binding(output_dir / name) for name in output_names],
        "generator": _binding(Path(__file__)),
        "safety": {
            "held_out_real_test_consumed": False,
            "held_out_test_receipt_exists": False,
            "formal_p3_result_mutated": False,
            "p4_qualification_result_mutated": False,
        },
        "pending": [],
    }
    _write_json(output_dir / "evidence_manifest_v1.json", manifest)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    manifest = generate(arguments.output_dir)
    print(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
