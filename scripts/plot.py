# (c) 2021 Philipp Ruppel

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import scipy.optimize
import glob
import random
import math

plt.rcParams["figure.figsize"] = (7,4)

colors = cm.rainbow(np.linspace(0, 1, len(sys.argv) / 2))

for i in range(len(sys.argv) / 2):
    
    fname = sys.argv[i * 2 + 1]
    label = sys.argv[i * 2 + 2]

    data = np.genfromtxt(fname)

    plt.plot(
        [(row[0] - data[0,0]) * (1.0 / 60.0) for row in data], 
        [math.sqrt(row[1]) for row in data],
        label=label,
        linewidth=2,
        c=colors[i]
        )

plt.xlim([-0.25,5.25])

plt.xlabel("Training time [min]")
plt.ylabel("Loss [RMSE]")

plt.legend()

plt.tight_layout()

plt.savefig("lossplot.png")
plt.savefig("lossplot.pdf")

plt.show()
