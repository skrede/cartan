#!/usr/bin/env python3
"""The two supplier-boundary rules: the pin's own integrity, and who may cross it.

A version argument or a mutable revision reintroduces the drift the pin exists
to prevent; a direct pugixml dependency or a supplier link outside cartan_urdf
widens the boundary the URDF module exists to hold. Both are build-file
properties, so both scan the same tracked-file set the eval-option rule reads.
"""

import re

MEIOS_LINK_TARGET = "cartan_urdf"

VERSIONED_FIND_PACKAGE = ("a version argument on the supplier's find_package call constrains what "
                          "an installed or enclosing-project meios may be, but the pinned revision "
                          "below is the only authority over which meios this repository builds; a "
                          "version written from the branch that revision sits on can refuse a "
                          "correct installation, or accept one the revision was never tested against")
UNPINNED_REVISION = ("CARTAN_MEIOS_REVISION is not a full 40-character hex commit SHA, so it names a "
                     "tag or a branch instead of a fixed point in the supplier's history; a mutable "
                     "ref reintroduces the drift the pin exists to prevent")
DIRECT_PUGIXML = ("pugixml enters cartan only transitively, through the supplier's own "
                  "find_dependency; a direct reference here puts it back in the set a consumer "
                  "has to satisfy before it can link cartan at all")
STRAY_MEIOS_LINK = (f"only {MEIOS_LINK_TARGET} may name the supplier; every other target reaching the "
                    "description reader through a transitive link is exactly the coupling the URDF "
                    "module exists to contain")

FIND_PACKAGE_MEIOS = re.compile(r"find_package\s*\(\s*(?:meios|\$\{CARTAN_MEIOS_PACKAGE\})\b([^)]*)\)",
                                re.IGNORECASE)
FIND_PACKAGE_VERSION = re.compile(r"^\s*[0-9]+(?:\.[0-9]+){0,3}\b")
REVISION_ASSIGNMENT = re.compile(r"\bset\s*\(\s*CARTAN_MEIOS_REVISION\s+([^\s)]+)")
FULL_SHA = re.compile(r"^[0-9a-fA-F]{40}$")
PUGIXML_TOKEN = re.compile(r"\bpugixml\b", re.IGNORECASE)
TARGET_LINK_LIBRARIES = re.compile(r"\btarget_link_libraries\s*\(\s*([A-Za-z0-9_]+)")
MEIOS_NAMESPACE = re.compile(r"\bmeios::")


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


def matching_close(text: str, opening: int) -> int:
    depth, index = 0, opening
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
                return index
        index += 1
    return len(text)


def find_package_findings(body: str):
    for match in FIND_PACKAGE_MEIOS.finditer(body):
        if FIND_PACKAGE_VERSION.match(match.group(1)):
            yield line_of(body, match.start()), match.group(), VERSIONED_FIND_PACKAGE


def revision_findings(body: str):
    for match in REVISION_ASSIGNMENT.finditer(body):
        if not FULL_SHA.match(match.group(1)):
            yield line_of(body, match.start()), match.group(1), UNPINNED_REVISION


def pugixml_findings(body: str):
    """One finding per line: the namespaced spelling carries the token twice."""
    reported = set()
    for match in PUGIXML_TOKEN.finditer(body):
        line = line_of(body, match.start())
        if line not in reported:
            reported.add(line)
            yield line, match.group(), DIRECT_PUGIXML


def meios_link_findings(body: str):
    for match in TARGET_LINK_LIBRARIES.finditer(body):
        target = match.group(1)
        opening = body.index("(", match.start())
        call_body = body[opening:matching_close(body, opening)]
        if target != MEIOS_LINK_TARGET and MEIOS_NAMESPACE.search(call_body):
            yield line_of(body, match.start()), f"target_link_libraries({target} ...)", \
                STRAY_MEIOS_LINK
