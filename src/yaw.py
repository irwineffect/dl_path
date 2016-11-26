#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf
from math import pi


class Node:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback, queue_size=1)

        self.yaw_pub = rospy.Publisher('yaw', Float32, queue_size=1)

    def callback(self, odom):
        q = odom.pose.pose.orientation
        (_,_,yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_pub.publish(yaw*(180.0/pi))
        


if __name__ == "__main__":
    rospy.init_node("yaw_publisher")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()

