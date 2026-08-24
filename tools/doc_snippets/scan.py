"""Walk one markdown document and record what each of its fences is marked as."""

from . import sentinel
from .sentinel import SnippetError

# Fragment preamble used when a page carries no <!-- cartan:preamble --> block.
DEFAULT_PREAMBLE = "\n".join(
    (
        "#include <cartan/serial_chain.h>",
        "#include <cartan/analytical.h>",
        "#include <Eigen/Core>",
        "#include <iostream>",
    )
)


def new_document() -> dict:
    return {
        "preamble": None,
        "snippets": [],
        "recipes": [],
        "unclassified": [],
        "cpp_fences": 0,
        "unbuilt": {kind: 0 for kind in sentinel.UNBUILT_KINDS},
    }


def read_fence(lines: list[str], open_index: int, where: str) -> tuple[list[str], int]:
    """Collect a fenced block's body. Return (body_lines, index_after_the_close)."""
    body: list[str] = []
    index = open_index + 1
    while index < len(lines):
        if sentinel.FENCE_CLOSE_RE.match(lines[index]):
            return body, index + 1
        body.append(lines[index])
        index += 1
    raise SnippetError(f"{where}: unterminated code fence")


def wrap_fragment(fragment: str, preamble: str) -> str:
    indented = "\n".join(
        ("    " + line) if line.strip() else line for line in fragment.splitlines()
    )
    return f"{preamble}\n\nint main()\n{{\n{indented}\n    (void)0;\n}}\n"


def take_preamble(state: dict, spec: dict, body: list[str], where: str) -> None:
    state["preamble"] = "\n".join(body)


def take_snippet(state: dict, spec: dict, body: list[str], where: str) -> None:
    block = "\n".join(body)
    if spec["tu"]:
        source = block if block.endswith("\n") else block + "\n"
    else:
        preamble = state["preamble"] if state["preamble"] is not None else DEFAULT_PREAMBLE
        source = wrap_fragment(block, preamble)
    state["snippets"].append({"name": spec["name"], "needs": spec["needs"], "source": source})


def take_unbuilt(state: dict, spec: dict, body: list[str], where: str) -> None:
    state["unbuilt"][spec["kind"]] += 1


def take_recipe(state: dict, spec: dict, body: list[str], where: str) -> None:
    """Record a recipe verbatim: what a later gate executes must be what is published."""
    state["recipes"].append(
        {"name": spec["name"], "kind": spec["kind"], "body": "\n".join(body) + "\n"}
    )


_TAKE = {
    "preamble": take_preamble,
    "snippet": take_snippet,
    "unbuilt": take_unbuilt,
    "recipe": take_recipe,
}


def take_fence(state: dict, pending, lang: str, body: list[str], where: str) -> None:
    if lang == "cpp":
        state["cpp_fences"] += 1
    if pending is None:
        if lang == "cpp":
            state["unclassified"].append(where)
        return
    kind, spec = pending
    shown = lang or "<none>"
    if kind in ("preamble", "snippet") and lang != "cpp":
        raise SnippetError(f"{where}: a cartan:{kind} sentinel must precede a cpp fence, got {shown}")
    if kind == "recipe" and lang == "cpp":
        raise SnippetError(f"{where}: a cartan:recipe sentinel promises execution, not compilation")
    _TAKE[kind](state, spec, body, where)


def scan(text: str, label: str) -> dict:
    """Collect every fence in one document, classified by the sentinel above it."""
    lines = text.splitlines()
    state = new_document()
    pending = None
    index = 0
    while index < len(lines):
        where = f"{label}:{index + 1}"
        match = sentinel.SENTINEL_RE.match(lines[index].strip())
        fence = sentinel.FENCE_OPEN_RE.match(lines[index])
        if match:
            if pending is not None:
                raise SnippetError(f"{where}: a sentinel is not followed by a code fence")
            pending = (match.group("kind"), sentinel.parse(match.group("kind"), match.group("attrs"), where))
        elif fence:
            body, index = read_fence(lines, index, where)
            take_fence(state, pending, fence.group("lang").lower(), body, where)
            pending = None
            continue
        elif pending is not None and lines[index].strip():
            raise SnippetError(f"{where}: a cartan:{pending[0]} sentinel is not followed by a code fence")
        index += 1
    if pending is not None:
        raise SnippetError(f"{label}: a trailing cartan:{pending[0]} sentinel has no fence")
    return state
