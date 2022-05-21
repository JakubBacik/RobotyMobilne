#!/usr/bin/env python

#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and display them on the screen

import rospy
from sensor_msgs.msg import CompressedImage
import cv2, cv_bridge
class Follower:
        def __init__(self):
                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)
                self.image_sub = rospy.Subscriber('rm3/camera/image_raw/compressed',
                CompressedImage, self.image_callback)

        def image_callback(self, msg):
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                cv2.imshow("window", image)
                cv2.waitKey(3)
rospy.init_node('follower')
follower = Follower()
rospy.spin()
