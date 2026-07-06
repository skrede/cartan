#!/usr/bin/env python3
"""Extract marked C++ documentation snippets into compilable translation units.

The documentation carries HTML-comment sentinels that mark which fenced ``cpp``
blocks are meant to compile against the real headers:

    <!-- cartan:preamble -->
    ```cpp
    #include <cartan/serial_chain.h>
    using Chain = cartan::kinematic_chain<double, 6>;
    ```

    <!-- cartan:snippet name=single-policy -->
    ```cpp
    cartan::basic_ik_runner solver{cartan::lm<Chain, cartan::no_limits>{}};
    ```

Sentinel grammar:

  * ``<!-- cartan:preamble -->`` before a ``cpp`` fence records that file's
    fragment preamble (includes + using-aliases prepended to every following
    fragment in the same file).
  * ``<!-- cartan:snippet name=<id> [tu] [needs=argmin|nlopt|urdf] -->`` before a
    ``cpp`` fence marks a block for compilation.
      - ``tu``   -> the block is a complete translation unit, emitted verbatim.
      - default  -> the block is a fragment, wrapped with the file preamble (or a
                    sensible default) and an ``int main()`` body.
      - ``needs=<cap>`` -> the snippet is grouped under <cap> and compiled only
                    when the matching backend option is enabled by the build.
  * Unmarked fences are ignored.

A malformed sentinel is a hard error (non-zero exit) so the gate fails loudly
rather than silently dropping a snippet. Output is deterministic: files are
walked and emitted in sorted order.

Standard library only -- no third-party imports.
"""

import re
import sys
import argparse
from pathlib import Path

CAPABILITIES = ("argmin", "nlopt", "urdf")
CORE = "core"

# Default fragment preamble used when a file has no <!-- cartan:preamble --> block.
DEFAULT_PREAMBLE = "\n".join(
    (
        "#include <cartan/serial_chain.h>",
        "#include <cartan/analytical.h>",
        "#include <Eigen/Core>",
        "#include <iostream>",
    )
)

# A sentinel occupies its own line: an HTML comment whose body starts with
# "cartan:". The kind is "snippet" or "preamble"; the remainder is the attribute
# string, parsed separately.
_SENTINEL_RE = re.compile(r"^<!--\s*cartan:(?P<kind>snippet|preamble)\b(?P<attrs>.*?)-->\s*$")

# Fenced-code delimiters. The opening fence may carry a language tag; the closing
# fence is bare.
_FENCE_OPEN_RE = re.compile(r"^```(?P<lang>[A-Za-z0-9_+-]*)\s*$")
_FENCE_CLOSE_RE = re.compile(r"^```\s*$")

# A snippet name is a slug: letters, digits, hyphen, underscore.
_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class SnippetError(Exception):
    """A malformed sentinel or structural error in the doc corpus."""


def _slug(rel_path: Path) -> str:
    """Turn a docs-relative markdown path into a filename-safe slug."""
    stem = rel_path.with_suffix("")
    return "-".join(stem.parts)


def _parse_snippet_attrs(attrs: str, where: str) -> dict:
    """Parse the attribute string of a snippet sentinel into a spec dict."""
    name = None
    is_tu = False
    needs = None
    for token in attrs.split():
        if token == "tu":
            if is_tu:
                raise SnippetError(f"{where}: duplicate 'tu' flag")
            is_tu = True
        elif token.startswith("name="):
            if name is not None:
                raise SnippetError(f"{where}: duplicate 'name=' attribute")
            name = token[len("name=") :]
            if not _NAME_RE.match(name):
                raise SnippetError(f"{where}: invalid snippet name {name!r}")
        elif token.startswith("needs="):
            if needs is not None:
                raise SnippetError(f"{where}: duplicate 'needs=' attribute")
            needs = token[len("needs=") :]
            if needs not in CAPABILITIES:
                raise SnippetError(
                    f"{where}: unknown capability {needs!r} "
                    f"(expected one of {', '.join(CAPABILITIES)})"
                )
        else:
            raise SnippetError(f"{where}: unrecognized snippet attribute {token!r}")
    if name is None:
        raise SnippetError(f"{where}: snippet sentinel missing required 'name=' attribute")
    return {"name": name, "tu": is_tu, "needs": needs}


def _read_fence(lines: list[str], open_index: int, where: str) -> tuple[list[str], int]:
    """Collect the body of a fenced block. Return (body_lines, index_after_close)."""
    body: list[str] = []
    j = open_index + 1
    while j < len(lines):
        if _FENCE_CLOSE_RE.match(lines[j]):
            return body, j + 1
        body.append(lines[j])
        j += 1
    raise SnippetError(f"{where}: unterminated code fence")


def _wrap_fragment(fragment: str, preamble: str) -> str:
    """Wrap a fragment body in the file preamble and an int main()."""
    indented = "\n".join(
        ("    " + line) if line.strip() else line for line in fragment.splitlines()
    )
    return f"{preamble}\n\nint main()\n{{\n{indented}\n    (void)0;\n}}\n"


def parse_markdown(text: str, rel_path: Path) -> tuple[list[dict], int]:
    """Extract snippet specs from one markdown document.

    Returns (snippets, skipped_fence_count). Each snippet is a dict with keys
    name, needs, source (the emitted C++). skipped_fence_count is informational
    only (unmarked cpp fences).
    """
    lines = text.splitlines()
    preamble = None
    pending = None  # ("preamble", None) | ("snippet", spec)
    snippets: list[dict] = []
    unmarked_cpp = 0

    i = 0
    while i < len(lines):
        line = lines[i]
        m = _SENTINEL_RE.match(line.strip())
        if m:
            where = f"{rel_path}:{i + 1}"
            if pending is not None:
                raise SnippetError(f"{where}: sentinel is not immediately followed by a code fence")
            if m.group("kind") == "preamble":
                pending = ("preamble", None)
            else:
                pending = ("snippet", _parse_snippet_attrs(m.group("attrs"), where))
            i += 1
            continue

        fm = _FENCE_OPEN_RE.match(line)
        if fm:
            where = f"{rel_path}:{i + 1}"
            body, after = _read_fence(lines, i, where)
            lang = fm.group("lang").lower()
            if pending is None:
                if lang == "cpp":
                    unmarked_cpp += 1
                i = after
                continue
            kind, spec = pending
            pending = None
            if lang != "cpp":
                raise SnippetError(
                    f"{where}: cartan:{kind} sentinel must precede a ```cpp fence, got ```{lang or '<none>'}"
                )
            block = "\n".join(body)
            if kind == "preamble":
                preamble = block
            else:
                assert spec is not None  # snippet kind always carries a parsed spec
                if spec["tu"]:
                    source = block if block.endswith("\n") else block + "\n"
                else:
                    source = _wrap_fragment(block, preamble if preamble is not None else DEFAULT_PREAMBLE)
                snippets.append(
                    {"name": spec["name"], "needs": spec["needs"], "source": source}
                )
            i = after
            continue

        # A non-fence, non-sentinel line. A pending sentinel followed by prose is
        # malformed; blank lines between sentinel and fence are tolerated.
        if pending is not None and line.strip() != "":
            kind = pending[0]
            raise SnippetError(
                f"{rel_path}:{i + 1}: cartan:{kind} sentinel is not immediately followed by a code fence"
            )
        i += 1

    if pending is not None:
        raise SnippetError(f"{rel_path}: trailing cartan:{pending[0]} sentinel with no following fence")

    return snippets, unmarked_cpp


def extract(docs_root: Path, out_dir: Path) -> dict:
    """Walk docs_root, emit one .cpp per marked snippet under out_dir, write a manifest.

    Returns a summary dict.
    """
    md_files = sorted(docs_root.rglob("*.md"))
    manifest: list[tuple[str, str]] = []  # (capability, relative_cpp_path)
    seen_names: dict[tuple[str, str], str] = {}
    counts = {CORE: 0, "argmin": 0, "nlopt": 0, "urdf": 0}
    skipped_unmarked = 0

    for md in md_files:
        rel = md.relative_to(docs_root)
        text = md.read_text(encoding="utf-8")
        snippets, unmarked = parse_markdown(text, rel)
        skipped_unmarked += unmarked
        slug = _slug(rel)
        for snip in snippets:
            capability = snip["needs"] or CORE
            filename = f"{slug}__{snip['name']}.cpp"
            key = (slug, snip["name"])
            if key in seen_names:
                raise SnippetError(
                    f"{rel}: duplicate snippet name {snip['name']!r} in the same document"
                )
            seen_names[key] = filename
            dest_dir = out_dir / capability
            dest_dir.mkdir(parents=True, exist_ok=True)
            (dest_dir / filename).write_text(snip["source"], encoding="utf-8")
            manifest.append((capability, f"{capability}/{filename}"))
            counts[capability] += 1

    # Ensure every capability directory exists so CMake's file(GLOB) never trips
    # on a missing path, even when a corpus has no snippet for that backend.
    for capability in (CORE, *CAPABILITIES):
        (out_dir / capability).mkdir(parents=True, exist_ok=True)

    manifest.sort()
    manifest_lines = [f"{cap}\t{path}" for cap, path in manifest]
    (out_dir / "manifest.txt").write_text(
        "\n".join(manifest_lines) + ("\n" if manifest_lines else ""), encoding="utf-8"
    )

    return {
        "counts": counts,
        "total": sum(counts.values()),
        "skipped_unmarked": skipped_unmarked,
        "manifest": str(out_dir / "manifest.txt"),
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Extract marked C++ documentation snippets into compilable translation units."
    )
    parser.add_argument(
        "--docs-root",
        type=Path,
        default=Path("docs"),
        help="Root directory to scan for markdown (default: docs)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output directory for generated .cpp files and manifest",
    )
    args = parser.parse_args(argv)

    if not args.docs_root.is_dir():
        print(f"error: docs root {args.docs_root} is not a directory", file=sys.stderr)
        return 2

    try:
        summary = extract(args.docs_root, args.out)
    except SnippetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    counts = summary["counts"]
    print(
        "extracted {total} snippet(s): "
        "core={core}, argmin={argmin}, nlopt={nlopt}, urdf={urdf}; "
        "ignored {skipped} unmarked cpp fence(s)".format(
            total=summary["total"],
            core=counts[CORE],
            argmin=counts["argmin"],
            nlopt=counts["nlopt"],
            urdf=counts["urdf"],
            skipped=summary["skipped_unmarked"],
        )
    )
    print(f"manifest: {summary['manifest']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
