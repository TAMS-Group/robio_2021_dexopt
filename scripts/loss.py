# (c) 2021 Philipp Ruppel

import fileinput
import time
import sys
import os

t0 = time.time()

f = open(sys.argv[1], "w")

inf = os.fdopen(sys.stdin.fileno(), 'r', 10)

lt = time.time()

while True:
    line = inf.readline()
    if "loss " in line and time.time() - lt > 1.0:
        lt = time.time()
        print time.time() - t0, line.strip().replace("loss ", "")
        f.write(str(time.time()))
        f.write(" ")
        f.write(line.strip().replace("loss ", ""))
        f.write("\n")
        f.flush()
