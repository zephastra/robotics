import collections
import hashlib
import json
import re

f = json.load(open("config/p3_freeze.json", encoding="utf-8"))
print("frozen artefact hashes vs disk:")
for a in f["artefacts"]:
    digest = hashlib.sha256(open(a["path"], "rb").read()).hexdigest()
    flag = "OK " if digest == a["sha256"] else "MISMATCH"
    print("  %-38s %s %s" % (a["path"], digest[:16], flag))

t = open("docs/DECISIONS.md", encoding="utf-8").read()
ids = [int(i[1:]) for i in re.findall(r"^##\s+(D\d+)\b", t, re.M)]
gaps = [b for a, b in zip(ids, ids[1:]) if b != a + 1]
dupes = [k for k, v in collections.Counter(ids).items() if v > 1]
print()
print("decisions D%d..D%d = %d entries, gaps %s, duplicates %s"
      % (ids[0], ids[-1], len(ids), gaps or "none", dupes or "none"))

board = open("docs/TASK_BOARD.md", encoding="utf-8").read()
row = next(l for l in board.split("\n") if l.startswith("| P4-HUMAN-01 |"))
print("board row status:", row.split("|")[2].strip())
print("board row carries note 19:", "\u2472" in row)
caps = open("docs/CAPABILITIES.md", encoding="utf-8").read()
print("capabilities:", [l.split("·")[0].strip() for l in caps.split("\n") if l.startswith("## C-")])
