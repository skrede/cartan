#!/usr/bin/env python3
"""Which constructs cartan refuses, over which files, and why each one is refused.

Separated from the driver because the two families span two languages and two
file sets; one flat pattern list would read as if the build rules were bolted
onto a C++ tool.
"""

import re
from pathlib import Path

CXX_EXTENSIONS = frozenset({".h", ".hh", ".hpp", ".hxx", ".inl", ".ipp",
                            ".c", ".cc", ".cpp", ".cxx"})
CXX_AREAS = ("lib/", "python/", "examples/", "tests/")
BUILD_NAMES = frozenset({"CMakeLists.txt", "CMakePresets.json"})
BUILD_SUFFIX = ".cmake"

EVAL_OPTION = "MEIOS_BUILD_EVAL_PYTHON"
TRUE_VALUES = frozenset({"ON", "TRUE", "YES", "Y", "1"})
FALSE_VALUES = frozenset({"OFF", "FALSE", "NO", "N", "0"})

SINKLESS_LOAD = ("this overload takes no log sink, so every diagnostic below the error tier is "
                 "discarded before the caller can see it; call load(path, options, log)")
LOAD_INTO = ("load_into's failure arm forwards one of the five fields its error carries, so a "
             "caller is told a load failed and handed nothing describing what")
ASSET_SKIP = ("skip suppresses the unresolved-asset diagnostic and clears the completeness claim "
              "recording the gap, so a caller receives a model asserting a completeness it lacks")
EVAL_ENABLED = ("enabling the description reader's Python evaluation backend in the build makes an "
                "expression inside an untrusted description executable for a user who never asked")
EVAL_UNPINNED = (f"no tracked build file assigns {EVAL_OPTION} a false value, so the build inherits "
                 "whatever default the pinned supplier revision happens to ship; restore "
                 f"set({EVAL_OPTION} OFF) in the supplier acquisition block")

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT = re.compile(r"//[^\n]*")
HASH_COMMENT = re.compile(r"#[^\n]*")
PROSE_STRING = re.compile(r'"[^"\n]*\s[^"\n]*"')
WORD = re.compile(r"[A-Za-z0-9_]+")

LOAD_CALL = re.compile(r"\bmeios::load\s*\(")
LOAD_INTO_CALL = re.compile(r"\bload_into\s*\(")
SKIP_TOKEN = re.compile(r"\bmissing_asset::skip\b")
OPTION_NAME = re.compile(rf"\b{EVAL_OPTION}\b")


def in_cxx_scope(path: str) -> bool:
    return Path(path).suffix in CXX_EXTENSIONS and path.startswith(CXX_AREAS)


def in_build_scope(path: str) -> bool:
    name = Path(path)
    return name.name in BUILD_NAMES or name.suffix == BUILD_SUFFIX


def blanked(match: re.Match) -> str:
    """Replace a span with its own newlines so every later line number still holds."""
    return "\n" * match.group().count("\n")


def strip_cxx_comments(text: str) -> str:
    return LINE_COMMENT.sub("", BLOCK_COMMENT.sub(blanked, text))


def strip_hash_comments(text: str) -> str:
    return HASH_COMMENT.sub("", text)


def line_of(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def end_of_literal(text: str, start: int) -> int:
    quote, index = text[start], start + 1
    while index < len(text):
        if text[index] == "\\":
            index += 2
        elif text[index] == quote:
            return index + 1
        else:
            index += 1
    return index


def argument_count(text: str, opening: int) -> int:
    """Count the top-level arguments of the call whose '(' sits at opening.

    Angle brackets are not tracked, so a template argument list carrying a comma
    counts high -- which errs toward accepting a call rather than refusing one.
    """
    depth, count, index = 0, 1, opening
    while index < len(text):
        char = text[index]
        if char in "\"'":
            index = end_of_literal(text, index)
            continue
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
            if depth == 0:
                return count if text[opening + 1:index].strip() else 0
        elif char == "," and depth == 1:
            count += 1
        index += 1
    return count


def cxx_findings(text: str):
    body = strip_cxx_comments(text)
    for match in LOAD_INTO_CALL.finditer(body):
        yield line_of(body, match.start()), match.group(), LOAD_INTO
    for match in SKIP_TOKEN.finditer(body):
        yield line_of(body, match.start()), match.group(), ASSET_SKIP
    for match in LOAD_CALL.finditer(body):
        arity = argument_count(body, match.end() - 1)
        if arity <= 2:
            yield line_of(body, match.start()), f"{match.group()} with {arity} argument(s)", \
                SINKLESS_LOAD


def assigned_value(remainder: str) -> str | None:
    """Classify the value an assignment gives the option, ignoring documentation prose.

    A quoted span carrying whitespace is a documentation string; a quoted single
    token is a value, which is how a preset writes one.
    """
    for word in WORD.findall(PROSE_STRING.sub(" ", remainder)):
        if word.upper() in TRUE_VALUES:
            return "true"
        if word.upper() in FALSE_VALUES:
            return "false"
    return None


def option_assignments(text: str):
    body = strip_hash_comments(text)
    for match in OPTION_NAME.finditer(body):
        tail = body[match.end():].split("\n", 1)[0]
        yield line_of(body, match.start()), tail, assigned_value(tail)


def build_findings(text: str):
    for line, tail, value in option_assignments(text):
        if value == "true":
            yield line, f"{EVAL_OPTION}{tail}".strip(), EVAL_ENABLED


def pins_option(text: str) -> bool:
    return any(value == "false" for _, _, value in option_assignments(text))
