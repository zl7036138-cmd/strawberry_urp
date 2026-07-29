#!/usr/bin/env python3
"""Assemble and receipt the field-v3 submission demonstration video."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from typing import Any

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = ROOT / "results" / "submission" / "field_v3_demo_v13"
DEFAULT_OUTPUT = (
    ROOT
    / "artifacts"
    / "submission_v2"
    / "video"
    / "strawberry_urp_submission_v2.mp4"
)
WIDTH = 1280
HEIGHT = 720
FPS = 10
LIVE_DURATION_SEC = 180.0
REPEAT_TRIALS = {
    "v15c": (
        "results/development/field_v3_perception_pick_v15c/trial_01.json",
        "44d98f669260d488352e2f2192a5bde021d8dc1baa14303844859763cf3b8a4c",
    ),
    "v16": (
        "results/development/field_v3_perception_pick_v16/trial_01.json",
        "06099efb7b49b2cd7f1d35978d7f3f65ce5db6216fbbd9728ffdeef540543074",
    ),
    "v17": (
        "results/development/field_v3_perception_pick_v17/trial_01.json",
        "11b492152b358ef1302e64609432e9f79b664a3c8ab4b3490759329ffb5b5b8d",
    ),
}
REQUIRED_STAGES = [
    "TARGET_READY",
    "PLAN",
    "APPROACH",
    "GRASP",
    "RETREAT",
    "PLACE",
    "VERIFY",
    "DONE",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def _binding(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"required video input is missing: {path}")
    return {
        "path": _relative(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _font(size: int, *, latin: bool = False) -> ImageFont.FreeTypeFont:
    name = "arial.ttf" if latin else "simhei.ttf"
    return ImageFont.truetype(f"C:/Windows/Fonts/{name}", size=size)


def _slide(
    path: Path,
    title: str,
    lines: list[str],
    *,
    accent: tuple[int, int, int],
    footer: str,
) -> None:
    image = Image.new("RGB", (WIDTH, HEIGHT), (16, 22, 30))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 24, HEIGHT), fill=accent)
    draw.rectangle((58, 82, 1180, 86), fill=accent)
    draw.text((60, 30), title, font=_font(38), fill=(244, 247, 250))
    y = 126
    for line in lines:
        section = line.startswith("◆")
        latin = not any(ord(char) > 127 for char in line)
        draw.text(
            (82, y),
            line,
            font=_font(29 if section else 25, latin=latin),
            fill=accent if section else (221, 229, 237),
        )
        y += 54 if section else 45
    draw.line((58, 650, 1220, 650), fill=(72, 84, 98), width=2)
    draw.text((60, 670), footer, font=_font(18), fill=(145, 163, 182))
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def _run(command: list[str]) -> None:
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"{completed.stdout}\n{completed.stderr}"
        )


def _probe(ffprobe: str, path: Path) -> dict[str, Any]:
    completed = subprocess.run(
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
            str(path),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(completed.stdout)


def _validate(run_dir: Path) -> tuple[Path, Path, Path, dict[str, Any]]:
    raw = run_dir / "field_v3_live_demo.avi"
    recording_path = run_dir / "field_v3_live_demo.receipt.json"
    trial_path = run_dir / "trial_01.json"
    recording = _json(recording_path)
    trial = _json(trial_path)
    if not trial.get("success") or not trial.get("result_success"):
        raise ValueError("field-v3 submission trial did not complete successfully")
    observations = recording.get("observations") or {}
    if observations.get("outcomes") != ["SUCCESS"]:
        raise ValueError("field-v3 recording does not contain final SUCCESS")
    stages = observations.get("stages") or []
    if stages != REQUIRED_STAGES:
        raise ValueError(f"recorded action stages differ: {stages}")
    for key in ("base_images", "wrist_images", "target_messages"):
        if int(observations.get(key) or 0) <= 0:
            raise ValueError(f"recording has no {key}")
    contacts = trial["diagnostics"]["contacts"]
    if not all(contacts[side]["processed_seen_true"] for side in ("left", "right")):
        raise ValueError("trial does not contain bilateral processed contact")
    events = trial["diagnostics"]["attachment_state"]["events"]
    if [event["attached"] for event in events] != [True, False]:
        raise ValueError("trial attachment sequence is not attach then detach")
    if _sha256(raw) != recording["video"]["sha256"]:
        raise ValueError("raw recording hash differs from its receipt")
    return raw, recording_path, trial_path, trial


def assemble(
    output: Path,
    run_dir: Path,
    work_dir: Path,
) -> dict[str, Any]:
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise ValueError("ffmpeg and ffprobe must be available on PATH")
    raw, recording_path, trial_path, trial = _validate(run_dir.resolve())

    repeat_bindings: dict[str, dict[str, Any]] = {}
    for name, (relative, expected_hash) in REPEAT_TRIALS.items():
        path = ROOT / relative
        if _sha256(path) != expected_hash:
            raise ValueError(f"{name} result hash differs")
        if not _json(path).get("success"):
            raise ValueError(f"{name} is not successful")
        repeat_bindings[name] = _binding(path)

    metrics_path = ROOT / "artifacts" / "p5" / "release_v1" / "final_metrics_v1.json"
    metrics = _json(metrics_path)
    if metrics["formal_p3"]["positive_successes"] != 39:
        raise ValueError("formal P3 numerator differs")
    if metrics["formal_p3"]["positive_trials"] != 135:
        raise ValueError("formal P3 denominator differs")
    if metrics["formal_p3"]["negative_safe_no_picks"] != 30:
        raise ValueError("formal negative result differs")

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
                "ROS 2 Jazzy  ·  Gazebo Harmonic  ·  YOLO11s  ·  MoveIt 2",
                "",
                "真实植株模型场景 + 底座全局相机 + 夹爪近距 RGB-D 相机",
                "成熟度检测 → 三维定位 → 安全预抓取 → 抓取与放置",
            ],
            (74, 174, 225),
            "提交演示｜固定 field-v3 场景｜仅仿真",
        ),
        (
            "02_architecture",
            18,
            "双相机协同与系统主线",
            [
                "◆ 底座相机",
                "提供植株、机械臂与作业区的全局观察",
                "◆ 夹爪相机",
                "YOLO 检测 + RGB-D 深度 + TF 坐标变换得到三维目标",
                "◆ 执行安全门",
                "姿态稳定 → 碰撞场景完整 → 预抓取可规划 → 才允许动作",
            ],
            (74, 184, 133),
            "顺序协同，不宣称多相机融合",
        ),
        (
            "04_repeat",
            20,
            "field-v3 可重复性与证据绑定",
            [
                "固定场景、固定 target 1、固定 seed",
                "v15c：SUCCESS + 双指接触 + attach/detach",
                "v16：SUCCESS + 双指接触 + attach/detach",
                "v17：SUCCESS + 双指接触 + attach/detach",
                "本视频：新录制的完整 SUCCESS 运行",
            ],
            (74, 184, 133),
            "开发演示证据；三份历史结果由 SHA-256 固定",
        ),
        (
            "05_formal",
            22,
            "正式指标：通过项与未通过项并列保留",
            [
                "YOLO 接受模型：F1 = 0.800675（内部目标 0.85，未达到）",
                "正式 P3 成熟果：39/135 = 28.89%（门槛 80%，未通过）",
                "仅未成熟负例：30/30 安全 NO_PICK",
                "P4 重遮挡检测：300/300；三维目标位姿：0/300",
                "本演示不替代正式 P3/P4 科学门",
            ],
            (231, 116, 76),
            "失败门不重写、不隐藏；提交重点是完整工程闭环与可复核性",
        ),
        (
            "06_conclusion",
            18,
            "提交结论与适用边界",
            [
                "◆ 已完成",
                "植株场景、双相机、视觉三维定位、安全规划和抓放闭环",
                "正式报告、复现实验、演示视频与确定性提交包",
                "◆ 明确限制",
                "无实体机器人、果实损伤、果柄剪切或 sim-to-real 结论",
            ],
            (74, 174, 225),
            "本科生 URP 提交版｜工程闭环完成，科学局限如实呈现",
        ),
    ]

    segments: list[Path] = []

    def make_slide(item: tuple[Any, ...]) -> None:
        name, duration, title, lines, accent, footer = item
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
        segments.append(segment)

    make_slide(slides[0])
    make_slide(slides[1])
    raw_probe = _probe(ffprobe, raw)
    raw_duration = float(raw_probe["format"]["duration"])
    if raw_duration <= 0.0:
        raise ValueError("raw field-v3 recording duration is invalid")
    live = work_dir / "03_field_v3_live.mp4"
    _run(
        [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(raw),
            "-an",
            "-vf",
            (
                f"setpts={LIVE_DURATION_SEC / raw_duration:.9f}*PTS,"
                f"scale={WIDTH}:{HEIGHT},fps={FPS}"
            ),
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "21",
            "-pix_fmt",
            "yuv420p",
            str(live),
        ]
    )
    segments.append(live)
    for item in slides[2:]:
        make_slide(item)

    concat = work_dir / "concat.txt"
    concat.write_text(
        "".join(f"file '{path.as_posix()}'\n" for path in segments),
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

    metadata = _probe(ffprobe, output)
    duration = float(metadata["format"]["duration"])
    stream = metadata["streams"][0]
    if not 240.0 <= duration <= 360.0:
        raise ValueError(f"final duration is outside 4-6 minutes: {duration}")
    if stream["codec_name"] != "h264":
        raise ValueError("final video is not H.264")
    if (stream["width"], stream["height"]) != (WIDTH, HEIGHT):
        raise ValueError("final video dimensions differ")

    receipt = {
        "schema_version": 1,
        "kind": "submission_v2_demonstration_video",
        "status": "PASS",
        "video": {
            "path": _relative(output),
            "size_bytes": output.stat().st_size,
            "sha256": _sha256(output),
            "duration_sec": duration,
            "codec": stream["codec_name"],
            "width": stream["width"],
            "height": stream["height"],
            "average_frame_rate": stream["avg_frame_rate"],
            "audio": False,
        },
        "live_segment": {
            "source_duration_sec": raw_duration,
            "presentation_duration_sec": LIVE_DURATION_SEC,
            "speed_factor": raw_duration / LIVE_DURATION_SEC,
            "outcome": "SUCCESS",
            "target_id": trial["target_id"],
            "stages": REQUIRED_STAGES,
        },
        "bindings": {
            "raw_video": _binding(raw),
            "raw_recording_receipt": _binding(recording_path),
            "trial": _binding(trial_path),
            "repeat_trials": repeat_bindings,
            "formal_metrics": _binding(metrics_path),
            "storyboard": _binding(ROOT / "docs/submission-video-storyboard-v2.md"),
            "report_source": _binding(ROOT / "docs/submission-report.md"),
        },
        "safety": {
            "formal_evidence": False,
            "fixed_scene": True,
            "fixed_target": True,
            "formal_p3_failure_preserved": True,
            "p4_failure_preserved": True,
            "held_out_real_test_consumed": False,
            "physical_robot_evidence": False,
            "fruit_damage_evidence": False,
            "sim_to_real_claim": False,
        },
    }
    receipt_path = output.with_suffix(".receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=ROOT / ".codex_tmp" / "submission_video_v2",
    )
    arguments = parser.parse_args()
    result = assemble(arguments.output, arguments.run_dir, arguments.work_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
