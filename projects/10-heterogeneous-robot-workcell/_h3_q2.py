lines = open("_h3_board.py", encoding="utf-8").read().split("\n")
print("total lines", len(lines))
for k in range(16, min(len(lines), 70)):
    l = lines[k]
    first = repr(l[:2])
    last = repr(l[-2:])
    flag = ""
    if not l.startswith("    '"):
        flag = "  <-- does not start with a quoted string"
    print("%3d first=%-10s last=%-8s %s" % (k + 1, first, last, flag))
