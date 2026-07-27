#!/usr/bin/env python3
"""The rules EXCEPTIONS.md encodes: which files it governs, and what makes a row valid."""

import re
from pathlib import Path

CEILING = 200
GROWTH_NUMERATOR = 11
GROWTH_DENOMINATOR = 10
REGISTRY_NAME = "EXCEPTIONS.md"
EXTENSIONS = frozenset({".h", ".hh", ".hpp", ".hxx", ".inl", ".ipp", ".c", ".cc", ".cpp", ".cxx"})
EXCLUDED_PREFIXES = ("benchmarks/third_party/",)

SEPARATOR = re.compile(r"^\|(?:\s*:?-+:?\s*\|)+$")
COUNT_IN_REASON = re.compile(r"\b\d+\s*(?:lines?|-line)\b")
SPLIT_HARM = re.compile(r"split|decompos|separat|scatter", re.IGNORECASE)
FUTURE_WORK = re.compile(r"\b(?:planned|will be|to be split|todo|for now|eventually|later)\b",
                         re.IGNORECASE)


def in_scope(path: str) -> bool:
    return Path(path).suffix in EXTENSIONS and not path.startswith(EXCLUDED_PREFIXES)


def allowance(recorded: int) -> int:
    return (recorded * GROWTH_NUMERATOR + GROWTH_DENOMINATOR - 1) // GROWTH_DENOMINATOR


def registry_rows(lines: list[str]):
    """Every table line that is neither a separator nor a header is a row and must parse."""
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped.startswith("|") or SEPARATOR.match(stripped):
            continue
        following = lines[index + 1].strip() if index + 1 < len(lines) else ""
        if SEPARATOR.match(following):
            continue
        yield index + 1, [cell.strip().strip("`").strip() for cell in stripped.strip("|").split("|")]


def reason_fault(cells: list[str], where: str, measured: int | None) -> str | None:
    reason = cells[2]
    counts = {cells[1]} | ({str(measured)} if measured is not None else set())
    if not reason:
        return f"{where}: the reason is empty, which is a silent exception with extra steps"
    if COUNT_IN_REASON.search(reason) or any(re.search(rf"\b{n}\b", reason) for n in counts):
        return f"{where}: the reason restates the file's line count instead of justifying it"
    if not SPLIT_HARM.search(reason):
        return f"{where}: the reason must name what splitting this file would harm"
    if FUTURE_WORK.search(reason):
        return f"{where}: the reason promises future work; this registers decisions already taken"
    return None


def row_fault(cells: list[str], number: int, counts: dict[str, int]) -> str | None:
    where = f"{REGISTRY_NAME} line {number}, {cells[0] or '<no path>'}"
    if len(cells) < 3 or not cells[0]:
        return f"{where}: a row needs a path, a recorded line count and a reason"
    if not in_scope(cells[0]):
        return f"{where}: the path is outside the checked scope"
    if not cells[1].isdigit():
        return f"{where}: the recorded count column is not a number"
    return reason_fault(cells, where, counts.get(cells[0]))


def parse_registry(text: str, counts: dict[str, int]) -> tuple[dict[str, int], list[str]]:
    recorded: dict[str, int] = {}
    faults: list[str] = []
    for number, cells in registry_rows(text.splitlines()):
        fault = row_fault(cells, number, counts)
        if fault is not None:
            faults.append(fault)
        elif cells[0] in recorded:
            faults.append(f"{REGISTRY_NAME} line {number}, {cells[0]}: the path is registered twice")
        else:
            recorded[cells[0]] = int(cells[1])
    return recorded, faults
