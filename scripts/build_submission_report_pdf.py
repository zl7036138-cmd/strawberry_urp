#!/usr/bin/env python3
"""Build the formal submission PDF from the same Markdown source as the DOCX."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
from typing import Any

from pypdf import PdfReader
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    Preformatted,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

from build_submission_report import _parse_blocks


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "docs" / "submission-report.md"
OVERVIEW = ROOT / "artifacts" / "submission_v2" / "figures" / "field_v3_overview.png"
WRIST = (
    ROOT
    / "results"
    / "development"
    / "field_v3_perception_shadow_v6"
    / "wrist_rgb.png"
)
DEFAULT_OUTPUT = (
    ROOT
    / "artifacts"
    / "submission_v2"
    / "report"
    / "草莓采摘URP项目总结报告.pdf"
)
BLUE = colors.HexColor("#185A8D")
DARK_BLUE = colors.HexColor("#17324D")
MUTED = colors.HexColor("#687684")
TABLE_FILL = colors.HexColor("#DCEAF4")
TABLE_ALT = colors.HexColor("#F4F8FB")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _text(value: str) -> str:
    return html.escape(value.replace("`", "").replace("**", "").strip())


def _register_fonts() -> None:
    pdfmetrics.registerFont(TTFont("CNBody", "C:/Windows/Fonts/simsun.ttc", subfontIndex=0))
    pdfmetrics.registerFont(TTFont("CNHead", "C:/Windows/Fonts/simhei.ttf"))
    pdfmetrics.registerFont(TTFont("Code", "C:/Windows/Fonts/consola.ttf"))


def _styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "body": ParagraphStyle(
            "CNBody",
            parent=base["BodyText"],
            fontName="CNBody",
            fontSize=10.5,
            leading=17,
            textColor=colors.HexColor("#222222"),
            alignment=TA_JUSTIFY,
            spaceAfter=5,
            wordWrap="CJK",
        ),
        "h1": ParagraphStyle(
            "CNH1",
            parent=base["Heading1"],
            fontName="CNHead",
            fontSize=17,
            leading=23,
            textColor=DARK_BLUE,
            spaceBefore=12,
            spaceAfter=8,
            keepWithNext=True,
            wordWrap="CJK",
        ),
        "h2": ParagraphStyle(
            "CNH2",
            parent=base["Heading2"],
            fontName="CNHead",
            fontSize=13,
            leading=19,
            textColor=BLUE,
            spaceBefore=9,
            spaceAfter=6,
            keepWithNext=True,
            wordWrap="CJK",
        ),
        "caption": ParagraphStyle(
            "CNCaption",
            parent=base["BodyText"],
            fontName="CNBody",
            fontSize=8.5,
            leading=12,
            textColor=MUTED,
            alignment=TA_CENTER,
            spaceAfter=8,
            wordWrap="CJK",
        ),
        "cover_small": ParagraphStyle(
            "CoverSmall",
            parent=base["BodyText"],
            fontName="CNHead",
            fontSize=12,
            leading=18,
            textColor=BLUE,
            alignment=TA_CENTER,
        ),
        "cover_title": ParagraphStyle(
            "CoverTitle",
            parent=base["Title"],
            fontName="CNHead",
            fontSize=23,
            leading=34,
            textColor=DARK_BLUE,
            alignment=TA_CENTER,
            wordWrap="CJK",
        ),
        "cover_subtitle": ParagraphStyle(
            "CoverSubtitle",
            parent=base["BodyText"],
            fontName="CNHead",
            fontSize=14,
            leading=20,
            textColor=MUTED,
            alignment=TA_CENTER,
        ),
        "table": ParagraphStyle(
            "CNTable",
            parent=base["BodyText"],
            fontName="CNBody",
            fontSize=8.2,
            leading=11.5,
            textColor=colors.HexColor("#202020"),
            alignment=TA_LEFT,
            wordWrap="CJK",
        ),
        "code": ParagraphStyle(
            "Code",
            parent=base["Code"],
            fontName="Code",
            fontSize=7.8,
            leading=10.5,
            leftIndent=4,
            rightIndent=4,
            textColor=colors.HexColor("#222222"),
        ),
    }


def _page(canvas, document) -> None:
    number = canvas.getPageNumber()
    if number == 1:
        return
    canvas.saveState()
    canvas.setFont("CNBody", 8)
    canvas.setFillColor(MUTED)
    canvas.drawString(
        document.leftMargin,
        A4[1] - 13 * mm,
        "中国农业大学本科生 URP｜草莓采摘仿真项目总结报告",
    )
    canvas.drawRightString(A4[0] - document.rightMargin, 11 * mm, f"第 {number} 页")
    canvas.setStrokeColor(colors.HexColor("#B8C2CC"))
    canvas.setLineWidth(0.4)
    canvas.line(
        document.leftMargin,
        A4[1] - 15 * mm,
        A4[0] - document.rightMargin,
        A4[1] - 15 * mm,
    )
    canvas.restoreState()


def _cover(story: list[Any], styles: dict[str, ParagraphStyle]) -> None:
    story.extend(
        [
            Spacer(1, 16 * mm),
            Paragraph("中国农业大学本科生 URP", styles["cover_small"]),
            Spacer(1, 8 * mm),
            Paragraph(
                "基于 ROS 2、YOLO 与 Gazebo 的<br/>"
                "草莓成熟度识别及机械臂抓取搬运仿真",
                styles["cover_title"],
            ),
            Spacer(1, 5 * mm),
            Paragraph("提交版项目总结报告", styles["cover_subtitle"]),
            Spacer(1, 8 * mm),
            Image(str(OVERVIEW), width=164 * mm, height=83 * mm),
            Spacer(1, 6 * mm),
        ]
    )
    rows = [
        ["项目类型", "本科生 URP"],
        ["当前范围", "仿真系统、双相机观察、三维定位与抓取搬运"],
        ["提交日期", "2026-07-29"],
        ["证据原则", "正式失败门保留；field-v3 成功为非正式开发演示"],
    ]
    table = Table(
        [[Paragraph(_text(cell), styles["table"]) for cell in row] for row in rows],
        colWidths=[34 * mm, 130 * mm],
    )
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (0, -1), TABLE_FILL),
                ("FONTNAME", (0, 0), (-1, -1), "CNBody"),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#AEB8C4")),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
            ]
        )
    )
    story.extend([table, PageBreak()])


def build(output: Path) -> dict[str, Any]:
    _register_fonts()
    styles = _styles()
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    document = SimpleDocTemplate(
        str(output),
        pagesize=A4,
        leftMargin=22 * mm,
        rightMargin=22 * mm,
        topMargin=22 * mm,
        bottomMargin=18 * mm,
        title="草莓采摘 URP 项目总结报告",
        author="草莓 URP 项目组",
    )
    story: list[Any] = []
    _cover(story, styles)
    lines = SOURCE.read_text(encoding="utf-8").splitlines()
    start = next(index for index, line in enumerate(lines) if line.strip() == "## 摘要")
    previous_kind: str | None = None
    number = 0
    inserted_overview = False
    inserted_wrist = False
    for kind, value in _parse_blocks(lines[start:]):
        if kind == "h1":
            story.append(Paragraph(_text(str(value)), styles["h1"]))
        elif kind == "h2":
            story.append(Paragraph(_text(str(value)), styles["h2"]))
        elif kind == "paragraph":
            text = str(value)
            story.append(Paragraph(_text(text), styles["body"]))
            if not inserted_overview and text.startswith("field-v3 来自用户提供的"):
                story.extend(
                    [
                        Spacer(1, 2 * mm),
                        Image(str(OVERVIEW), width=160 * mm, height=81 * mm),
                        Paragraph("图 1  field-v3 草莓田场景总览", styles["caption"]),
                    ]
                )
                inserted_overview = True
            if not inserted_wrist and text.startswith("固定目标的无运动检查"):
                story.extend(
                    [
                        Spacer(1, 2 * mm),
                        Image(str(WRIST), width=142 * mm, height=106.5 * mm),
                        Paragraph(
                            "图 2  腕部 RGB-D 相机的 field-v3 目标视图",
                            styles["caption"],
                        ),
                    ]
                )
                inserted_wrist = True
        elif kind in {"bullet", "number"}:
            if kind == "number":
                if previous_kind != "number":
                    number = 0
                number += 1
                bullet = f"{number}."
            else:
                bullet = "◆"
            style = ParagraphStyle(
                f"list-{kind}",
                parent=styles["body"],
                leftIndent=8 * mm,
                firstLineIndent=-5 * mm,
                spaceAfter=4,
            )
            story.append(
                Paragraph(
                    f"<font name='CNHead'>{bullet}</font>&nbsp;&nbsp;{_text(str(value))}",
                    style,
                )
            )
        elif kind == "code":
            story.append(
                Table(
                    [[Preformatted(str(value), styles["code"])]],
                    colWidths=[164 * mm],
                    style=TableStyle(
                        [
                            ("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#F2F4F7")),
                            ("BOX", (0, 0), (-1, -1), 0.4, colors.HexColor("#CDD4DB")),
                            ("LEFTPADDING", (0, 0), (-1, -1), 6),
                            ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                            ("TOPPADDING", (0, 0), (-1, -1), 5),
                            ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                        ]
                    ),
                )
            )
            story.append(Spacer(1, 2 * mm))
        elif kind == "table":
            rows = value
            if rows:
                count = len(rows[0])
                widths = {
                    2: [46 * mm, 118 * mm],
                    3: [44 * mm, 78 * mm, 42 * mm],
                    4: [34 * mm, 36 * mm, 34 * mm, 60 * mm],
                }.get(count, [164 * mm / count] * count)
                data = [
                    [Paragraph(_text(cell), styles["table"]) for cell in row]
                    for row in rows
                ]
                table = Table(data, colWidths=widths, repeatRows=1, splitByRow=1)
                commands = [
                    ("BACKGROUND", (0, 0), (-1, 0), TABLE_FILL),
                    ("FONTNAME", (0, 0), (-1, 0), "CNHead"),
                    ("GRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#AEB8C4")),
                    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                    ("LEFTPADDING", (0, 0), (-1, -1), 4),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                    ("TOPPADDING", (0, 0), (-1, -1), 4),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                ]
                for row_index in range(2, len(rows), 2):
                    commands.append(("BACKGROUND", (0, row_index), (-1, row_index), TABLE_ALT))
                table.setStyle(TableStyle(commands))
                story.extend([table, Spacer(1, 2 * mm)])
        previous_kind = kind
    document.build(story, onFirstPage=_page, onLaterPages=_page)
    pages = len(PdfReader(str(output)).pages)
    receipt = {
        "schema_version": 1,
        "kind": "submission_report_pdf_build",
        "preset": "narrative_proposal",
        "header_pattern": "editorial_cover",
        "source": {"path": SOURCE.relative_to(ROOT).as_posix(), "sha256": _sha256(SOURCE)},
        "figures": [
            {"path": OVERVIEW.relative_to(ROOT).as_posix(), "sha256": _sha256(OVERVIEW)},
            {"path": WRIST.relative_to(ROOT).as_posix(), "sha256": _sha256(WRIST)},
        ],
        "pdf": {
            "path": output.relative_to(ROOT).as_posix(),
            "size_bytes": output.stat().st_size,
            "sha256": _sha256(output),
            "page_count": pages,
        },
    }
    receipt_path = output.with_suffix(".pdf-build-receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    result = build(arguments.output)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
