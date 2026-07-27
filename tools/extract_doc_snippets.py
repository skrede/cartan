#!/usr/bin/env python3
"""Extract the fenced blocks the documentation publishes, and classify the rest.

Every fenced ``cpp`` block in the documentation says something about the API, so
every one of them carries a sentinel saying what it is:

    <!-- cartan:preamble -->            includes and aliases shared by a page
    <!-- cartan:snippet name=<id> -->   compiled against the real headers
    <!-- cartan:unbuilt kind=<k> -->    published but never built, and why

A ``snippet`` is emitted as a translation unit: ``tu`` marks a block that is
already a whole program, anything else is wrapped in the page preamble and an
``int main()``. ``needs=argmin|nlopt|urdf`` groups a snippet so the build
compiles it only where that component exists.

An ``unbuilt`` fence declares one of three classes. ``declaration`` is a
declaration extract copied from a header -- a signature, a class synopsis, an
enumeration -- whose useful check is whether the declaration still reads that
way, not whether a statement compiles. ``illustration`` is code written to make
a point rather than to build, and ``sketch`` is a shape rather than code; both
must say why in a ``reason`` attribute, because "this is not built" without a
stated why is the silence the classification exists to break.

A non-``cpp`` fence may carry ``<!-- cartan:recipe kind=cmake|shell name=<id> -->``.
Its body is written out verbatim, terminated by a newline, and listed in the
manifest so a consumption gate executes the published text rather than a copy
of it that has since drifted.

With ``--require-classified`` an unsentinelled ``cpp`` fence is a non-zero exit
naming the first one, and so is a corpus that holds no fence or marks none for
compilation. Without it the same fences are only counted, which is what a
contributor drafting a page wants and what a gate must never accept.

Standard library only -- no third-party imports.
"""

import sys
import argparse
from pathlib import Path

from doc_snippets import emit
from doc_snippets.sentinel import SnippetError

EXIT_SENTINEL = 1
EXIT_CORPUS = 2


def counted(counts: dict) -> str:
    return ", ".join(f"{key}={value}" for key, value in counts.items())


def report(summary: dict) -> None:
    print(
        "scanned {documents} document(s), {fences} cpp fence(s): "
        "compiled {compiled} ({groups}); unbuilt {unbuilt} ({classes}); "
        "recipes {recipes} ({kinds}); unclassified {open}".format(
            documents=summary["documents"],
            fences=summary["cpp_fences"],
            compiled=sum(summary["snippets"].values()),
            groups=counted(summary["snippets"]),
            unbuilt=sum(summary["unbuilt"].values()),
            classes=counted(summary["unbuilt"]),
            recipes=sum(summary["recipes"].values()),
            kinds=counted(summary["recipes"]),
            open=len(summary["unclassified"]),
        )
    )
    print(f"manifest: {summary['manifest']}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract and classify the fenced blocks published in the documentation."
    )
    parser.add_argument("--docs-root", type=Path, default=Path("docs"),
                        help="directory scanned recursively for markdown (default: docs)")
    parser.add_argument("--extra-file", type=Path, action="append", default=[], dest="extra_files",
                        help="an individual markdown file outside the root; repeatable")
    parser.add_argument("--require-classified", action="store_true",
                        help="refuse a cpp fence that carries no sentinel, and refuse an empty corpus")
    parser.add_argument("--out", type=Path, required=True,
                        help="output directory for the generated files and the manifest")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    if not args.docs_root.is_dir():
        print(f"error: the docs root {args.docs_root} is not a directory", file=sys.stderr)
        return EXIT_CORPUS
    missing = [str(path) for path in args.extra_files if not path.is_file()]
    if missing:
        print(f"error: extra file(s) not found: {', '.join(missing)}", file=sys.stderr)
        return EXIT_CORPUS
    try:
        summary = emit.extract(args.docs_root, args.out, args.extra_files, args.require_classified)
    except SnippetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_SENTINEL
    report(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
