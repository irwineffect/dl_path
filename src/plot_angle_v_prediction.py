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
    plt.plot(vals[:,0], vals[:,2])
    plt.plot(vals[:,0], vals[:,3])
    plt.legend(['left', 'forward', 'right'], loc="best")
    plt.xticks(np.arange(-90, 91, 10))
    plt.xlabel("robot angle (degrees)")
    plt.ylabel("class probability prediction")
    plt.title("Class Probability vs. Angle")
    plt.savefig("class_v_angle.png")
    plt.show()
    
