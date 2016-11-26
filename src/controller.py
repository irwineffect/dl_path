#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from dl_path.msg import lrf
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV
from classifier import Classifier


class Node:
    def __init__(self):
        self.forward_p = 0.3
        self.rotate_p = 0.5

        self.latest_image = None
        self.classifier = Classifier()
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('image', Image, self.image_callback, queue_size=1)

        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.prob_pub = rospy.Publisher('classes', lrf, queue_size=1)

    def run(self):
        if self.latest_image is None:
            return

        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(self.latest_image))
        cv2.imshow("image", cv_image)
        cv2.waitKey(1)
        
        probs = [0.,0.,0.]
        probs = self.classifier.classify(cv_image)
        class_msg = lrf()
        class_msg.left = probs[0]
        class_msg.forward = probs[1]
        class_msg.right = probs[2]

        self.prob_pub.publish(class_msg) 

        rospy.logdebug("probabilities:\nleft: {:1.3f}\nforward: {:1.3f}\nright: {:1.3f}".format(probs[0], probs[1], probs[2]))

        cmd = Twist()
        forward_certainty = probs[1]
        rotate_certainty = probs[2] - probs[0]

        cmd.linear.x = self.forward_p
        
        #cmd.angular.z = rotate_certainty * self.rotate_p 

        rotate_probs = [probs[0], probs[2]]
        
        max_rotate_index = np.argmax(rotate_probs)
        direction = [-1, 1]
        
        if rotate_probs[max_rotate_index] > 0.5:
            cmd.angular.z = direction[max_rotate_index] * self.rotate_p

        #publish command to the turtlebot
        self.movement_pub.publish(cmd)


        

    def image_callback(self, ros_image):
        self.latest_image = ros_image
        
if __name__ == "__main__":
    rospy.init_node("deeplearning_controller")
    node = Node()

    r = rospy.Rate(3)

    while not rospy.is_shutdown():
        node.run()
        r.sleep()
    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()

