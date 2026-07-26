"""Small, deterministic YAML reader for tracked Strawberry URP configuration.

The project configuration intentionally uses a narrow YAML subset: nested
string-keyed mappings, scalar values, and scalar block lists.  Keeping that
reader here lets the ROS-independent benchmark tools run on a clean Python
installation without importing PyYAML.  Unsupported YAML constructs fail
closed with a line-numbered error instead of being interpreted approximately.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any


_KEY_VALUE = re.compile(r"^([A-Za-z_][A-Za-z0-9_.-]*):(.*)$")
_INTEGER = re.compile(r"^[+-]?[0-9]+$")
_FLOAT = re.compile(
    r"^[+-]?(?:[0-9]+\.[0-9]*|[0-9]*\.[0-9]+|[0-9]+[eE][+-]?[0-9]+|"
    r"[0-9]+\.[0-9]*[eE][+-]?[0-9]+)$"
)


class _Token:
    __slots__ = ("indent", "content", "line_number")

    def __init__(self, indent: int, content: str, line_number: int) -> None:
        self.indent = indent
        self.content = content
        self.line_number = line_number


def _remove_comment(line: str) -> str:
    single_quoted = False
    double_quoted = False
    escaped = False
    for index, character in enumerate(line):
        if escaped:
            escaped = False
            continue
        if character == "\\" and double_quoted:
            escaped = True
            continue
        if character == "'" and not double_quoted:
            single_quoted = not single_quoted
            continue
        if character == '"' and not single_quoted:
            double_quoted = not double_quoted
            continue
        if character == "#" and not single_quoted and not double_quoted:
            return line[:index]
    return line


def _tokenize(text: str) -> list[_Token]:
    tokens: list[_Token] = []
    for line_number, original in enumerate(text.splitlines(), start=1):
        without_comment = _remove_comment(original).rstrip()
        if not without_comment.strip():
            continue
        prefix_length = len(without_comment) - len(without_comment.lstrip(" "))
        if "\t" in without_comment[:prefix_length]:
            raise ValueError(f"line {line_number}: tab indentation is not supported")
        tokens.append(
            _Token(prefix_length, without_comment[prefix_length:], line_number)
        )
    return tokens


def _scalar(text: str, line_number: int) -> Any:
    value = text.strip()
    if not value:
        raise ValueError(f"line {line_number}: an empty scalar is not supported")
    if value.startswith('"'):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError as exc:
            raise ValueError(f"line {line_number}: invalid quoted string") from exc
        if not isinstance(parsed, str):
            raise ValueError(f"line {line_number}: expected a quoted string")
        return parsed
    if value.startswith("'"):
        if len(value) < 2 or not value.endswith("'"):
            raise ValueError(f"line {line_number}: invalid quoted string")
        return value[1:-1].replace("''", "'")
    normalized = value.lower()
    if normalized in {"true", "false"}:
        return normalized == "true"
    if normalized in {"null", "~"}:
        return None
    if _INTEGER.fullmatch(value):
        return int(value)
    if _FLOAT.fullmatch(value):
        return float(value)
    if any(marker in value for marker in ("[", "]", "{", "}", "&", "*", "!")):
        raise ValueError(
            f"line {line_number}: unsupported YAML syntax in scalar {value!r}"
        )
    return value


def _parse_block(tokens: list[_Token], index: int, indent: int) -> tuple[Any, int]:
    if index >= len(tokens):
        raise ValueError("unexpected end of YAML input")
    is_list = tokens[index].content.startswith("-")
    container: Any = [] if is_list else {}

    while index < len(tokens):
        token = tokens[index]
        if token.indent < indent:
            break
        if token.indent > indent:
            raise ValueError(f"line {token.line_number}: unexpected indentation")

        if is_list:
            if not token.content.startswith("- "):
                raise ValueError(
                    f"line {token.line_number}: mappings and lists cannot share a block"
                )
            container.append(_scalar(token.content[2:], token.line_number))
            index += 1
            continue

        if token.content.startswith("-"):
            raise ValueError(
                f"line {token.line_number}: mappings and lists cannot share a block"
            )
        match = _KEY_VALUE.fullmatch(token.content)
        if match is None:
            raise ValueError(f"line {token.line_number}: expected 'key: value'")
        key, remainder = match.groups()
        if key in container:
            raise ValueError(f"line {token.line_number}: duplicate key {key!r}")
        index += 1
        if remainder.strip():
            container[key] = _scalar(remainder, token.line_number)
            continue
        if index >= len(tokens) or tokens[index].indent <= indent:
            raise ValueError(f"line {token.line_number}: key {key!r} has no value")
        child_indent = tokens[index].indent
        child, index = _parse_block(tokens, index, child_indent)
        container[key] = child

    return container, index


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    """Read the supported YAML subset and require a mapping at the root."""

    source = Path(path)
    tokens = _tokenize(source.read_text(encoding="utf-8"))
    if not tokens:
        raise ValueError(f"YAML config is empty: {source}")
    if tokens[0].indent != 0:
        raise ValueError(f"line {tokens[0].line_number}: root must not be indented")
    parsed, next_index = _parse_block(tokens, 0, 0)
    if next_index != len(tokens):
        token = tokens[next_index]
        raise ValueError(f"line {token.line_number}: content was not parsed")
    if not isinstance(parsed, dict):
        raise ValueError("YAML config root must be a mapping")
    return parsed
