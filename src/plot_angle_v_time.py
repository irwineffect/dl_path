#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np
import sys

if __name__ == "__main__":
    filename = sys.argv[1]
    #filename = "angle_predictions.csv"
    f = open(filename, "r")

    next(f)
    vals = []
    for line in f:
        line = [float(x) for x in line.strip('\n').split(',')]
        if (line[1] != 0):  
            vals.append(line)

    vals = np.array(vals)

    plt.plot(vals[:,0], vals[:,1])
    plt.plot(vals[:,0], np.zeros_like(vals[:,0]), '--')
    plt.legend(["heading", "zero-heading"], loc="upper right")
    plt.yticks(np.arange(-90, 91, 10))
    plt.ylabel("Heading (degrees)")
    plt.xlabel("Time (seconds)")
    plt.title("Heading vs. Time")

    outfilename = filename.strip(".csv") + ".png"
    print "saving to {}".format(outfilename)
    plt.savefig(outfilename)
    plt.show()
    
