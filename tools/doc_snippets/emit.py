"""Assemble the documentation corpus, enforce its classification, and write it out."""

from pathlib import Path

from . import scan, sentinel
from .sentinel import SnippetError

GROUPS = (sentinel.CORE, *sentinel.CAPABILITIES)


def root_slug(relative: Path) -> str:
    return "-".join(relative.with_suffix("").parts)


def documents(docs_root: Path, extra_files: list[Path]) -> list[tuple[str, str, Path]]:
    """The corpus as (label, slug, path) triples, root pages first, in a stable order."""
    corpus = [
        (page.relative_to(docs_root).as_posix(), root_slug(page.relative_to(docs_root)), page)
        for page in sorted(docs_root.rglob("*.md"))
    ]
    corpus += [(str(extra), f"extra-{extra.stem}", extra) for extra in extra_files]
    if not corpus:
        raise SnippetError(f"no markdown document was found under {docs_root} or among the extra files")
    return corpus


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise SnippetError(f"{path}: the document cannot be read as UTF-8 ({exc})") from exc


def place(out_dir: Path, group: str, filename: str, body: str, owners: dict, label: str) -> str:
    if filename in owners:
        raise SnippetError(
            f"{label}: the generated name {filename!r} collides with the one from {owners[filename]}"
        )
    owners[filename] = label
    (out_dir / group).mkdir(parents=True, exist_ok=True)
    (out_dir / group / filename).write_text(body, encoding="utf-8")
    return f"{group}/{filename}"


def write_document(out_dir: Path, label: str, slug: str, state: dict, owners: dict) -> list[tuple]:
    rows = []
    for snip in state["snippets"]:
        group = snip["needs"] or sentinel.CORE
        path = place(out_dir, group, f"{slug}__{snip['name']}.cpp", snip["source"], owners, label)
        rows.append(("snippet", group, snip["name"], path))
    for recipe in state["recipes"]:
        filename = f"{slug}__{recipe['name']}{sentinel.RECIPE_KINDS[recipe['kind']]}"
        path = place(out_dir, sentinel.RECIPE, filename, recipe["body"], owners, label)
        rows.append(("recipe", recipe["kind"], recipe["name"], path))
    return rows


def tally(summary: dict, state: dict) -> None:
    summary["cpp_fences"] += state["cpp_fences"]
    summary["unclassified"] += state["unclassified"]
    for kind, count in state["unbuilt"].items():
        summary["unbuilt"][kind] += count


def new_summary() -> dict:
    return {
        "documents": 0,
        "cpp_fences": 0,
        "unclassified": [],
        "unbuilt": {kind: 0 for kind in sentinel.UNBUILT_KINDS},
        "snippets": {group: 0 for group in GROUPS},
        "recipes": {kind: 0 for kind in sentinel.RECIPE_KINDS},
    }


def enforce(summary: dict) -> None:
    """Refuse an unclassified fence, and refuse a corpus that classified nothing."""
    unclassified = summary["unclassified"]
    if unclassified:
        raise SnippetError(
            f"{unclassified[0]}: this cpp fence carries no cartan sentinel"
            + (f", and {len(unclassified) - 1} further fence(s) carry none either" if len(unclassified) > 1 else "")
            + "; every cpp fence must carry a cartan:snippet, cartan:preamble or cartan:unbuilt sentinel"
        )
    if summary["cpp_fences"] == 0:
        raise SnippetError(
            f"the {summary['documents']} scanned document(s) hold no cpp fence at all: "
            "the corpus is empty, misrooted or unreadable"
        )
    if sum(summary["snippets"].values()) == 0:
        raise SnippetError(
            f"the corpus classified {summary['cpp_fences']} cpp fence(s) and marked none of them "
            "for compilation, so nothing would be checked against the headers"
        )


def extract(docs_root: Path, out_dir: Path, extra_files: list[Path], require_classified: bool) -> dict:
    """Emit one file per marked fence under out_dir and write the manifest beside them."""
    summary = new_summary()
    owners: dict[str, str] = {}
    rows: list[tuple] = []
    for label, slug, path in documents(docs_root, extra_files):
        state = scan.scan(read(path), label)
        summary["documents"] += 1
        tally(summary, state)
        rows += write_document(out_dir, label, slug, state, owners)
    for role, kind, _name, _path in rows:
        summary["snippets" if role == "snippet" else "recipes"][kind] += 1
    if require_classified:
        enforce(summary)
    for group in (*GROUPS, sentinel.RECIPE):
        (out_dir / group).mkdir(parents=True, exist_ok=True)
    manifest = out_dir / "manifest.txt"
    lines = ["\t".join(row) for row in sorted(rows)]
    manifest.write_text("".join(f"{line}\n" for line in lines), encoding="utf-8")
    summary["manifest"] = str(manifest)
    return summary
