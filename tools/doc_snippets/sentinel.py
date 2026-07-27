"""Sentinel grammar for the markers that classify a fenced documentation block.

A sentinel is an HTML comment on its own line immediately above a fence:

    <!-- cartan:preamble -->                         includes shared by a page
    <!-- cartan:snippet name=<id> [tu] [needs=<c>] --> compiled against the headers
    <!-- cartan:unbuilt kind=<k> [reason="..."] -->  published but never built
    <!-- cartan:recipe kind=<k> name=<id> -->        published to be executed

``reason`` holds a sentence and must therefore be quoted; an unquoted one is
rejected rather than silently truncated at the first space.
"""

import re
import shlex

CAPABILITIES = ("argmin", "nlopt", "urdf")
CORE = "core"
RECIPE = "recipe"

# The classes a published-but-unbuilt fence may declare, and whether the class
# alone says why the block is not built. A declaration extract is its own
# explanation; the other two are not, so they must carry a reason.
UNBUILT_KINDS = {"declaration": False, "illustration": True, "sketch": True}

RECIPE_KINDS = {"cmake": ".cmake", "shell": ".sh"}

KINDS = ("preamble", "snippet", "unbuilt", "recipe")

SENTINEL_RE = re.compile(r"^<!--\s*cartan:(?P<kind>[A-Za-z_-]+)\b(?P<attrs>.*?)-->\s*$")
FENCE_OPEN_RE = re.compile(r"^```(?P<lang>[A-Za-z0-9_+-]*)\s*$")
FENCE_CLOSE_RE = re.compile(r"^```\s*$")
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class SnippetError(Exception):
    """A malformed sentinel or a structural fault in the documentation corpus."""


def attributes(attrs: str, where: str, flags: tuple) -> tuple[dict, set]:
    """Split a sentinel's attribute string into key/value pairs and bare flags."""
    try:
        tokens = shlex.split(attrs)
    except ValueError as exc:
        raise SnippetError(f"{where}: unparsable sentinel attributes ({exc})") from exc
    values: dict[str, str] = {}
    present: set[str] = set()
    for token in tokens:
        key, separator, value = token.partition("=")
        if not separator:
            if token not in flags:
                raise SnippetError(f"{where}: unrecognized sentinel attribute {token!r}")
            present.add(token)
        elif key in values:
            raise SnippetError(f"{where}: duplicate {key!r} attribute")
        else:
            values[key] = value
    return values, present


def named(values: dict, key: str, where: str, allowed=None) -> str:
    """Read a required attribute, checking it against a closed vocabulary."""
    value = values.get(key)
    if value is None:
        raise SnippetError(f"{where}: sentinel missing required {key!r} attribute")
    if allowed is not None and value not in allowed:
        raise SnippetError(
            f"{where}: unknown {key} {value!r} (expected one of {', '.join(sorted(allowed))})"
        )
    return value


def only(values: dict, where: str, known: tuple) -> None:
    for key in values:
        if key not in known:
            raise SnippetError(f"{where}: unrecognized sentinel attribute {key!r}")


def slug_name(values: dict, where: str) -> str:
    name = named(values, "name", where)
    if not NAME_RE.match(name):
        raise SnippetError(f"{where}: invalid name {name!r} (letters, digits, '-' and '_' only)")
    return name


def snippet_spec(attrs: str, where: str) -> dict:
    values, flags = attributes(attrs, where, ("tu",))
    only(values, where, ("name", "needs"))
    needs = values.get("needs")
    if needs is not None and needs not in CAPABILITIES:
        raise SnippetError(
            f"{where}: unknown capability {needs!r} (expected one of {', '.join(CAPABILITIES)})"
        )
    return {"name": slug_name(values, where), "needs": needs, "tu": "tu" in flags}


def unbuilt_spec(attrs: str, where: str) -> dict:
    values, _ = attributes(attrs, where, ())
    only(values, where, ("kind", "reason"))
    kind = named(values, "kind", where, UNBUILT_KINDS)
    reason = values.get("reason", "").strip()
    if UNBUILT_KINDS[kind] and not reason:
        raise SnippetError(
            f"{where}: an unbuilt '{kind}' fence must carry a reason attribute; the class alone "
            "does not say why the block is not built"
        )
    return {"kind": kind, "reason": reason}


def recipe_spec(attrs: str, where: str) -> dict:
    values, _ = attributes(attrs, where, ())
    only(values, where, ("kind", "name"))
    return {"kind": named(values, "kind", where, RECIPE_KINDS), "name": slug_name(values, where)}


_SPECS = {"snippet": snippet_spec, "unbuilt": unbuilt_spec, "recipe": recipe_spec}


def parse(kind: str, attrs: str, where: str) -> dict:
    if kind not in KINDS:
        raise SnippetError(
            f"{where}: unknown sentinel 'cartan:{kind}' (expected one of {', '.join(KINDS)})"
        )
    if kind == "preamble":
        only(attributes(attrs, where, ())[0], where, ())
        return {}
    return _SPECS[kind](attrs, where)
