lines = open("_h3_board.py", encoding="utf-8").read().split("\n")
for k, l in enumerate(lines, 1):
    n = l.count("'")
    if n >= 4:
        print("line %d has %d single quotes:" % (k, n))
        print("   ", l)
