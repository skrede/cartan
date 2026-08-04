#!/usr/bin/env python3
"""Refuse the two ways the robot-descriptions guide could drift on evaluation trust.

The process-authority warning must appear verbatim, and only in the
unrestricted-backend section -- elsewhere it either does not warn where a
reader is about to reach for that backend, or it warns somewhere the built-in
or restricted backend is being described, which is not where the authority
grant happens. Endorsement language is refused everywhere on the page: the
guide describes trade-offs, it does not pick a winner.
"""

import re

DOCS_PATH = "docs/guides/robot-descriptions.md"

SENTENCE = "An expression inside a description evaluates with the process's own authority"
# Markdown hard-wraps prose across source lines, so the words of the sentence
# are not always separated by a single space in the file; a run of whitespace
# stands in for the space in the literal string being searched for.
SENTENCE_PATTERN = re.compile(r"\s+".join(re.escape(word) for word in SENTENCE.split()))
HEADING = re.compile(r"^#{2,3} .*$", re.MULTILINE)
UNRESTRICTED_HEADING = re.compile(r"^### The unrestricted Python backend$", re.MULTILINE)
ENDORSEMENT_LANGUAGE = re.compile(r"\brecommend\w*\b|\bprefer\w*\b|\buse the unrestricted\b",
                                  re.IGNORECASE)

SENTENCE_MISSING = f"'{SENTENCE}' does not appear in the unrestricted-backend section verbatim"
SENTENCE_MISPLACED = f"'{SENTENCE}' appears outside the unrestricted-backend section"
ENDORSEMENT_FOUND = "the page recommends or prefers the unrestricted evaluator over the restricted one"


def line_of(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def doc_findings(text: str):
    heading = UNRESTRICTED_HEADING.search(text)
    if not heading:
        yield 1, DOCS_PATH, SENTENCE_MISSING
        return
    section_end_match = HEADING.search(text, heading.end())
    section_end = section_end_match.start() if section_end_match else len(text)
    found_in_section = False
    for match in SENTENCE_PATTERN.finditer(text):
        index = match.start()
        if heading.end() <= index < section_end:
            found_in_section = True
        else:
            yield line_of(text, index), SENTENCE, SENTENCE_MISPLACED
    if not found_in_section:
        yield 1, DOCS_PATH, SENTENCE_MISSING
    for match in ENDORSEMENT_LANGUAGE.finditer(text):
        yield line_of(text, match.start()), match.group(), ENDORSEMENT_FOUND
