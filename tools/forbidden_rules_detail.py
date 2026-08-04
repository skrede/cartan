#!/usr/bin/env python3
"""Refuse the axis re-derivation the sink must not perform.

The real axis gate lives in build.h: it rejects a zero or overflowing axis
before a screw_axis is ever constructed. These rules are scoped to
lib/cartan-urdf/include/cartan/urdf/detail/ only -- one directory below and
one file over from build.h -- so they never fire on the gate itself. isfinite
is flagged only when its argument names an axis; a bare isfinite() check on an
unrelated scalar (a narrowing conversion, say) is not the construct this rule
exists to catch.
"""

import re

URDF_DETAIL_AREA = "lib/cartan-urdf/include/cartan/urdf/detail/"

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT = re.compile(r"//[^\n]*")

SQUARED_NORM_CALL = re.compile(r"\bsquaredNorm\s*\(")
AXIS_ISFINITE_CALL = re.compile(r"\bisfinite\s*\([^()]*\baxis\w*")
SCREW_AXIS_FACTORY = re.compile(r"\bscrew_axis\s*<[^>]*>\s*::\s*(revolute|prismatic)\s*\(")

DETAIL_AXIS_REASON = ("the axis gate lives in build.h, which normalizes and rejects a zero or "
                      "overflowing axis before a screw_axis is ever constructed; re-deriving that "
                      "check here duplicates a decision the chain builder already made and can "
                      "silently diverge from it")
DETAIL_FACTORY_REASON = ("screw_axis<...>::revolute/::prismatic return a bare axis through Eigen's "
                         "normalized(), which returns a zero vector unchanged rather than refusing "
                         "it; calling either factory here bypasses the axis gate in build.h")


def line_of(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def blanked(match: re.Match) -> str:
    return "\n" * match.group().count("\n")


def strip_cxx_comments(text: str) -> str:
    return LINE_COMMENT.sub("", BLOCK_COMMENT.sub(blanked, text))


def detail_findings(text: str):
    body = strip_cxx_comments(text)
    for match in SQUARED_NORM_CALL.finditer(body):
        yield line_of(body, match.start()), match.group(), DETAIL_AXIS_REASON
    for match in AXIS_ISFINITE_CALL.finditer(body):
        yield line_of(body, match.start()), match.group(), DETAIL_AXIS_REASON
    for match in SCREW_AXIS_FACTORY.finditer(body):
        yield line_of(body, match.start()), match.group(), DETAIL_FACTORY_REASON
