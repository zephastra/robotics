"""The two documents that carry the project's state, checked mechanically.

`D029` is why this file exists. It was cited twice from the task board and never written, and
nothing failed -- the gap was found by eye, a round later, and only because someone happened to
count. **A numbering scheme whose gaps are visible only to a person who is counting is not
checked at all.**

The separator is the trap this was written against, and it caught its own author first: 37
headings end `## Dnnn \u2014 ` and two end `## Dnnn\uff1a`, and a checker that requires a space after the
number reported "gaps [33, 34]" plus a dangling `D033` reference -- both artefacts of the regex,
not of the document. So the pattern stops at the digits and never assumes a separator.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DECISIONS = ROOT / 'docs' / 'DECISIONS.md'
BOARD = ROOT / 'docs' / 'TASK_BOARD.md'

HEADING = re.compile(r'^## D(\d{3})\b([^\n]*)', re.M)
REFERENCE = re.compile(r'`D(\d{3})`')


def decisions_text():
    return DECISIONS.read_text(encoding='utf-8')


def decision_ids():
    return [int(m.group(1)) for m in HEADING.finditer(decisions_text())]


def test_decision_numbers_are_unique_and_gapless():
    ids = decision_ids()
    duplicates = sorted({i for i in ids if ids.count(i) > 1})
    assert not duplicates, f'duplicate decision ids: {duplicates}'
    assert ids == sorted(ids), 'decisions are out of order'
    gaps = [n for n in range(ids[0], ids[-1] + 1) if n not in set(ids)]
    assert not gaps, (
        f'decision(s) {gaps} missing. A gap must be DECLARED as its own heading -- D029 is the '
        f'precedent: it carries the title "(空号：该决策未被写下...)". An undeclared gap is '
        f'invisible to everyone who is not counting the numbers themselves.')


def test_every_decision_heading_has_a_body():
    """A heading with nothing under it is a declared gap that forgot to say so."""
    text = decisions_text()
    matches = list(HEADING.finditer(text))
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        body = text[match.end():end].strip()
        assert body, f'D{match.group(1)} has a heading and no body'
        if '空号' not in match.group(2):
            assert len(body) > 40, (
                f'D{match.group(1)} declares a body of {len(body)} characters; either it is a '
                f'declared gap or it is a stub')


def test_every_decision_reference_resolves():
    ids = set(decision_ids())
    dangling = sorted({int(m) for m in REFERENCE.findall(decisions_text())} - ids)
    assert not dangling, f'DECISIONS.md cites {dangling}, which do not exist'


def test_every_decision_the_task_board_cites_exists():
    """This is the D029 case exactly: the board said "见 D028/D029/D030/D031-D036" while D029
    did not exist, and the citation read as proof that it did."""
    ids = set(decision_ids())
    cites = {int(m) for m in REFERENCE.findall(BOARD.read_text(encoding='utf-8'))}
    ranges = set()
    for low, high in re.findall(r'`D(\d{3})\s*[-–]\s*D?(\d{3})`', BOARD.read_text(encoding='utf-8')):
        ranges |= {n for n in range(int(low), int(high) + 1)}
    missing = sorted((cites | ranges) - ids)
    assert not missing, f'TASK_BOARD.md cites {missing}, which DECISIONS.md does not contain'


def test_no_task_id_appears_on_two_rows():
    """One id with two statuses is exactly how a stale row survives: `P2-C-ONION-08` was
    `PARTIAL` on one row and `DONE` on another, and both read as current."""
    rows = [line for line in BOARD.read_text(encoding='utf-8').splitlines()
            if line.startswith('|') and line.count('|') >= 4]
    ids = [row.split('|')[1].strip() for row in rows]
    ids = [name for name in ids if re.fullmatch(r'[A-Z][A-Z0-9]*(-[A-Z0-9]+)*', name)]
    duplicates = sorted({name for name in ids if ids.count(name) > 1})
    assert not duplicates, f'{duplicates} appear on more than one task board row'
