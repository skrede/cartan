#!/usr/bin/env python3
"""Refuse C++ attribute syntax the project's style rule bans.

Matching requires the closing '[[' ... ']]' to sit together with nothing but
attribute-list characters between them, which is what distinguishes real
attribute syntax from an immediately-invoked lambda used as a subscript (a
capture list closes with a single ']', not two) and from a nested subscript
chain (its brackets are never literally adjacent). Erring toward accepting a
construct rather than refusing it follows the discipline of argument_count in
forbidden_rules.py.
"""

import re

ATTRIBUTE_SYNTAX = re.compile(r"\[\[([^\[\]]*)\]\]")
ATTRIBUTE_USING = re.compile(r"^\s*using\s+\w+\s*:\s*(\w+)")
ATTRIBUTE_NAME = re.compile(r"^\s*(\w+(?:::\w+)?)")

# [[noreturn]] changes what the optimizer may assume about a function's control
# flow, and [[fallthrough]] / [[maybe_unused]] silence a warning the build
# promotes to an error; all three are load-bearing on the compiler. Nothing
# else in the standard attribute grammar is: [[nodiscard]] and the rest are
# decorative, which is what this project's rule refuses.
ALLOWED_ATTRIBUTES = frozenset({"noreturn", "fallthrough", "maybe_unused"})


def line_of(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def attribute_name(content: str) -> str:
    using = ATTRIBUTE_USING.match(content)
    if using:
        return using.group(1)
    name = ATTRIBUTE_NAME.match(content)
    return name.group(1) if name else content.strip()


def attribute_findings(body: str):
    for match in ATTRIBUTE_SYNTAX.finditer(body):
        name = attribute_name(match.group(1))
        if name.rsplit("::", 1)[-1] not in ALLOWED_ATTRIBUTES:
            yield line_of(body, match.start()), match.group(), \
                f"[[{name}]] is a decorative attribute the compiler does not require"
