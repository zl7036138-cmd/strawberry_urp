#!/usr/bin/env python3
"""Assemble and receipt the P5 simulation-only demonstration video."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from typing import Any

from PIL import Image, ImageDraw, ImageFont


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "artifacts" / "p5" / "video" / "strawberry_urp_demo_v1.mp4"
)
WIDTH = 1280
HEIGHT = 720
FPS = 10

POSITIVE = (
    "results/p5/headed_demo_capture_v4/run/camera_overlay_demo.mp4"
)
NEGATIVE = (
    "results/p5/headed_no_pick_capture_v1/run/camera_overlay_demo.mp4"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _binding(relative_path: str) -> dict[str, Any]:
    path = REPOSITORY_ROOT / relative_path
    if not path.is_file():
        raise ValueError(f"required video evidence is missing: {relative_path}")
    return {
        "path": relative_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _json(relative_path: str) -> dict[str, Any]:
    value = json.loads(
        (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")
    )
    if not isinstance(value, dict):
        raise ValueError(f"{relative_path} must contain a JSON object")
    return value


def _font(path: str, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(path, size=size)


def _slide(
    path: Path,
    title: str,
    lines: list[str],
    *,
    accent: tuple[int, int, int],
    footer: str,
) -> None:
    image = Image.new("RGB", (WIDTH, HEIGHT), (18, 24, 32))
    draw = ImageDraw.Draw(image)
    chinese_font = "C:/Windows/Fonts/simhei.ttf"
    latin_font = "C:/Windows/Fonts/arial.ttf"
    draw.rectangle((0, 0, 24, HEIGHT), fill=accent)
    draw.rectangle((55, 78, 1165, 82), fill=accent)
    draw.text((58, 28), title, font=_font(chinese_font, 38), fill=(244, 247, 250))
    y = 122
    for line in lines:
        is_section = line.startswith("◆")
        font_path = chinese_font if any(ord(char) > 127 for char in line) else latin_font
        font_size = 30 if is_section else 25
        fill = accent if is_section else (222, 229, 236)
        draw.text((78, y), line, font=_font(font_path, font_size), fill=fill)
        y += 55 if is_section else 44
    draw.rectangle((55, 652, 1225, 654), fill=(72, 84, 98))
    draw.text(
        (58, 670),
        footer,
        font=_font(chinese_font, 19),
        fill=(150, 166, 183),
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def _run(command: list[str]) -> None:
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"{completed.stdout}\n{completed.stderr}"
        )


def assemble(output: Path, work_dir: Path) -> dict[str, Any]:
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise ValueError("ffmpeg and ffprobe must be available on PATH")

    positive_trial = _json(
        "results/p5/headed_demo_capture_v4/run/trial.json"
    )
    negative_trial = _json(
        "results/p5/headed_no_pick_capture_v1/run/trial.json"
    )
    positive_receipt = _json(
        "results/p5/headed_demo_capture_v4/run/camera_overlay_receipt.json"
    )
    negative_receipt = _json(
        "results/p5/headed_no_pick_capture_v1/run/camera_overlay_receipt.json"
    )
    metrics = _json("artifacts/p5/release_v1/final_metrics_v1.json")
    clean_handoff = _json(
        "artifacts/p5/p5_clean_reproduction_handoff_v1.json"
    )
    if not positive_trial.get("success"):
        raise ValueError("headed positive demonstration did not pass")
    if positive_trial["final_status"]["outcome"] != "SUCCESS":
        raise ValueError("headed positive demonstration did not end in SUCCESS")
    if not negative_trial.get("success") or not negative_trial.get("safe_no_pick"):
        raise ValueError("headed only-unripe demonstration was not a safe NO_PICK")
    if negative_trial["final_status"]["outcome"] != "NO_PICK":
        raise ValueError("headed only-unripe demonstration outcome differs")
    if positive_receipt["observations"]["outcomes"] != ["SUCCESS"]:
        raise ValueError("positive recording does not contain SUCCESS")
    if negative_receipt["observations"]["outcomes"] != ["NO_PICK"]:
        raise ValueError("negative recording does not contain NO_PICK")
    if metrics["formal_p3"]["positive_successes"] != 39:
        raise ValueError("formal P3 numerator differs")
    if metrics["formal_p3"]["positive_trials"] != 135:
        raise ValueError("formal P3 denominator differs")
    if metrics["p4_qualification"]["heavy_target_pose_rate"] != 0.0:
        raise ValueError("P4 heavy target-pose result differs")
    if clean_handoff["status"] != "CLEAN_REPRODUCTION_ACCEPTED_VIDEO_PENDING":
        raise ValueError("clean reproduction handoff is not accepted")

    output = output.resolve()
    work_dir = work_dir.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True)

    slides = [
        (
            "01_title",
            12,
            "草莓成熟度识别与机械臂抓取搬运仿真",
            [
                "ROS 2 Jazzy  ·  Gazebo Harmonic  ·  MoveIt 2  ·  YOLO11s",
                "",
                "成熟度检测 → RGB-D 三维定位 → 运动规划 → 抓取放置",
                "",
                "项目演示与最终证据摘要",
            ],
            (70, 170, 225),
            "仿真研究｜不包含实体机器人或 sim-to-real 结论",
        ),
        (
            "02_architecture",
            15,
            "系统架构与证据边界",
            [
                "◆ 感知链",
                "RGB 图像 → YOLO 成熟/未成熟框 → 深度中位数 → TF 位姿",
                "◆ 执行链",
                "目标选择 → MoveIt 规划 → 接近/抓取/抬升/放置/验证",
                "◆ 本段抓放演示",
                "Oracle 控制机械臂；落选模型仅作 Shadow 可视化",
            ],
            (74, 184, 133),
            "非正式演示，不替代正式 135+30 场景矩阵",
        ),
        (
            "03_module",
            14,
            "已通过的模块门",
            [
                "T40 定位：100 个真值位置",
                "中位误差 1.345 mm；95 分位误差 1.897 mm",
                "T50 真值目标抓放：9/10（门槛 ≥90%）",
                "T60 Oracle 端到端：10/10；规划 p95 0.058841 s",
                "干净环境：7 个包，246 项测试全部通过",
            ],
            (74, 184, 133),
            "模块门通过不等于视觉驱动的正式 P3 通过",
        ),
        (
            "04_p3",
            16,
            "正式 P3 结果：未通过",
            [
                "135 次成熟果正式试验：39/135 = 28.89%",
                "验收门槛：≥80%",
                "无/部分/重遮挡：33/45、6/45、0/45",
                "失败归因：感知 84 次；抓取 12 次",
                "30 次仅未成熟负例：30/30 安全 NO_PICK",
            ],
            (231, 116, 76),
            "正式矩阵已消耗且不可重跑；结果由 ADR 0030 冻结",
        ),
        (
            "05_p4",
            16,
            "P4 单次干预：检测恢复，定位仍失败",
            [
                "重遮挡成熟检测：300/300 帧",
                "重遮挡目标位姿：0/300 帧",
                "原因：框中心深度落在前景遮挡物上",
                "最近真值距离约 0.345–0.366 m，超过 0.080 m 上限",
                "系统安全拒绝发布目标位姿；候选未获控制授权",
            ],
            (231, 116, 76),
            "ADR 0032 拒绝该干预并禁止第二次 P4 修复",
        ),
        (
            "06_reproduction",
            14,
            "P5 干净环境复现：通过",
            [
                "独立 WSL：Ubuntu-24.04-URP-Repro",
                "Ubuntu 24.04.4 + ROS 2 Jazzy + RTX 4060 CUDA",
                "7 个 ROS 包；246/246 测试通过",
                "5/5 成熟果成功；5/5 仅未成熟果 NO_PICK",
                "与参考环境行为通过率绝对差：0.0（上限 0.05）",
            ],
            (70, 170, 225),
            "ADR 0033 接受复现子门；真实图像测试集仍封存",
        ),
        (
            "07_conclusion",
            14,
            "结论与限制",
            [
                "◆ 已完成",
                "可复现 ROS 2 感知—定位—规划—抓放闭环与分层评测",
                "◆ 核心瓶颈",
                "视觉模型未达 0.85 F1；遮挡下中心深度定位失效",
                "◆ 不作出的主张",
                "无实体机器人、果实损伤、果柄剪切或 sim-to-real 结论",
            ],
            (70, 170, 225),
            "最终状态：发布完成，但正式 P3 与 P4 科学门保持失败",
        ),
    ]

    segment_paths: list[Path] = []
    for name, duration, title, lines, accent, footer in slides[:2]:
        png = work_dir / f"{name}.png"
        segment = work_dir / f"{name}.mp4"
        _slide(png, title, lines, accent=accent, footer=footer)
        _run(
            [
                ffmpeg,
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-loop",
                "1",
                "-i",
                str(png),
                "-t",
                str(duration),
                "-r",
                str(FPS),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                str(segment),
            ]
        )
        segment_paths.append(segment)

    def normalize(source: Path, target: Path) -> None:
        _run(
            [
                ffmpeg,
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-i",
                str(source),
                "-an",
                "-vf",
                f"scale={WIDTH}:{HEIGHT},fps={FPS}",
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "21",
                "-pix_fmt",
                "yuv420p",
                str(target),
            ]
        )

    positive_segment = work_dir / "03_positive_demo.mp4"
    normalize(REPOSITORY_ROOT / POSITIVE, positive_segment)
    segment_paths.append(positive_segment)

    negative_segment = work_dir / "04_negative_demo.mp4"
    normalize(REPOSITORY_ROOT / NEGATIVE, negative_segment)
    segment_paths.append(negative_segment)

    for name, duration, title, lines, accent, footer in slides[2:]:
        png = work_dir / f"{name}.png"
        segment = work_dir / f"{name}.mp4"
        _slide(png, title, lines, accent=accent, footer=footer)
        _run(
            [
                ffmpeg,
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-loop",
                "1",
                "-i",
                str(png),
                "-t",
                str(duration),
                "-r",
                str(FPS),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                str(segment),
            ]
        )
        segment_paths.append(segment)

    concat = work_dir / "concat.txt"
    concat.write_text(
        "".join(f"file '{path.as_posix()}'\n" for path in segment_paths),
        encoding="utf-8",
    )
    _run(
        [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(concat),
            "-c",
            "copy",
            "-movflags",
            "+faststart",
            str(output),
        ]
    )
    probe = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-show_entries",
            "stream=codec_name,width,height,avg_frame_rate",
            "-of",
            "json",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    metadata = json.loads(probe.stdout)
    duration = float(metadata["format"]["duration"])
    if not 240.0 <= duration <= 360.0:
        raise ValueError(f"final video duration is outside 4-6 minutes: {duration}")
    stream = metadata["streams"][0]
    if stream["codec_name"] != "h264":
        raise ValueError("final video codec is not H.264")
    if stream["width"] != WIDTH or stream["height"] != HEIGHT:
        raise ValueError("final video dimensions differ")

    receipt = {
        "schema_version": 1,
        "kind": "p5_final_demonstration_video",
        "status": "PASS",
        "video": {
            "path": output.relative_to(REPOSITORY_ROOT).as_posix(),
            "size_bytes": output.stat().st_size,
            "sha256": _sha256(output),
            "duration_sec": duration,
            "codec": stream["codec_name"],
            "width": stream["width"],
            "height": stream["height"],
            "average_frame_rate": stream["avg_frame_rate"],
            "audio": False,
        },
        "segments": [
            "title and architecture",
            "non-formal Oracle-controlled successful headed pick/place",
            "non-formal perception-controlled only-unripe safe NO_PICK",
            "accepted module metrics",
            "failed formal P3 result",
            "rejected P4 intervention and root cause",
            "accepted clean reproduction",
            "conclusions and limitations",
        ],
        "bindings": {
            key: _binding(path)
            for key, path in {
                "positive_video": POSITIVE,
                "positive_trial": (
                    "results/p5/headed_demo_capture_v4/run/trial.json"
                ),
                "positive_recording_receipt": (
                    "results/p5/headed_demo_capture_v4/run/"
                    "camera_overlay_receipt.json"
                ),
                "negative_video": NEGATIVE,
                "negative_trial": (
                    "results/p5/headed_no_pick_capture_v1/run/trial.json"
                ),
                "negative_recording_receipt": (
                    "results/p5/headed_no_pick_capture_v1/run/"
                    "camera_overlay_receipt.json"
                ),
                "release_metrics": (
                    "artifacts/p5/release_v1/final_metrics_v1.json"
                ),
                "clean_reproduction": (
                    "artifacts/p5/p5_clean_reproduction_handoff_v1.json"
                ),
                "storyboard": "docs/video-storyboard.md",
            }.items()
        },
        "safety": {
            "positive_demo_formal_evidence": False,
            "positive_demo_control_source": "ORACLE",
            "negative_demo_formal_evidence": False,
            "formal_p3_failure_preserved": True,
            "p4_failure_preserved": True,
            "held_out_real_test_consumed": False,
            "physical_robot_evidence": False,
            "sim_to_real_claim": False,
        },
    }
    source_snapshot = {
        "schema_version": 1,
        "kind": "p5_video_source_metrics_snapshot",
        "module_results": metrics["module_results"],
        "formal_p3": metrics["formal_p3"],
        "p4_qualification": metrics["p4_qualification"],
        "release_reproduction": metrics["release_reproduction"],
        "safety_and_scope": metrics["safety_and_scope"],
    }
    snapshot_path = output.with_name(f"{output.stem}.source-metrics.json")
    snapshot_path.write_text(
        json.dumps(
            source_snapshot, ensure_ascii=False, indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    receipt["bindings"].pop("release_metrics")
    receipt["bindings"]["source_metrics_snapshot"] = {
        "path": snapshot_path.relative_to(REPOSITORY_ROOT).as_posix(),
        "size_bytes": snapshot_path.stat().st_size,
        "sha256": _sha256(snapshot_path),
    }
    receipt_path = output.with_suffix(".receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=REPOSITORY_ROOT / ".codex_tmp" / "p5_video_v1",
    )
    arguments = parser.parse_args()
    receipt = assemble(arguments.output, arguments.work_dir)
    print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
