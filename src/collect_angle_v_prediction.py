#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from dl_path.msg import lrf
import tf
from math import pi

def myround(x, base=5):
    return int(base * round(float(x)/base))

class bin_value:
    def __init__(self):
        self.counts = dict()
        self.sums = dict()

        for i in xrange(-90, 91):
            self.counts[i] = 0
            self.sums[i] = 0.0

    def update(self, angle, value):
        angle = myround(angle, 5)
        if (angle >= -90) and (angle <= 90):
            self.counts[angle] += 1
            self.sums[angle] += value

    def get_averages(self):
        averages = dict()
        for i in xrange(-90, 91):
            if self.counts[i] == 0:
                averages[i] = 0
            else:
                averages[i] = self.sums[i]/self.counts[i]

        return averages
         
class Node:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('classes', lrf, self.prob_callback, queue_size=1)
        self.yaw_sub = rospy.Subscriber('yaw', Float32, self.angle_callback, queue_size=1)
        self.left_average = bin_value()
        self.forward_average = bin_value()
        self.right_average = bin_value()
        self.current_angle = 0.0

    def angle_callback(self, angle):
        self.current_angle = angle.data

    def prob_callback(self, lrf):
        angle = self.current_angle
        self.left_average.update(angle, lrf.left)
        self.forward_average.update(angle, lrf.forward)
        self.right_average.update(angle, lrf.right)


if __name__ == "__main__":
    rospy.init_node("angle_v_prediction_calculator")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
    
    left_averages = node.left_average.get_averages()
    right_averages = node.right_average.get_averages()
    forward_averages = node.forward_average.get_averages()

    f = open("angle_predictions.csv", 'w')
    f.write("angle, left, forward, right\n")
    for i in xrange(-90, 91):
        f.write("{}, {}, {}, {}\n".format(i, left_averages[i],
            forward_averages[i], right_averages[i]))

    f.close()
        
