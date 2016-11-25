#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV
from classifier import Classifier


class Node:
    def __init__(self):
        self.forward_p = 0.2
        self.rotate_p = 0.2

        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('image', Image, self.image_callback, queue_size=1)

        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.classifier = Classifier()

    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))
        cv2.imshow("image", cv_image)
        cv2.waitKey(1)
        
        probs = self.classifier.classify(cv_image)

        rospy.logdebug("probabilities:\nleft: {:1.3f}\nforward: {:1.3f}\nright: {:1.3f}".format(probs[0], probs[1], probs[2]))

        cmd = Twist()
        forward_certainty = probs[1]
        rotate_certainty = probs[0] - probs[2]

        cmd.linear.x = forward_certainty * self.forward_p

        cmd.angular.z = rotate_certainty * self.rotate_p 

        #publish command to the turtlebot
        self.movement_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("deeplearning_controller")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()

