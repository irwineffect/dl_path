#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import tf
from math import pi

class Node:
    def __init__(self):
        self.f = open("angle_v_time.csv", 'w')
        self.f.write("time, angle\n")

        self.base_time = None
        self.yaw_sub = rospy.Subscriber('yaw', Float32, self.angle_callback, queue_size=1)

    def angle_callback(self, angle):
        if  self.base_time is None:
            self.base_time = rospy.get_time()

        time = rospy.get_time()
        rtime = time - self.base_time
    
        self.f.write("{}, {}\n".format(rtime, angle.data))


if __name__ == "__main__":
    rospy.init_node("angle_v_time_logger")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
    
    node.f.close()
        
