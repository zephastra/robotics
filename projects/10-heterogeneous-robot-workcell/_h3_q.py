import io

lines = open("_h3_board.py", encoding="utf-8").read().split("\n")
for k, l in enumerate(lines, 1):
    n = l.count("'")
    if n % 2:
        print("ODD quote count %d on line %d:" % (n, k))
        print("   ", l[:160])
