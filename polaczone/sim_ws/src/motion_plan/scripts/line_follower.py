#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)

        self.image_sub = rospy.Subscriber('rm3/camera/image_raw',
                                          Image, self.image_callback)

        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.twist = Twist()

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image=image[0:800,100:700]
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        global err
        global errtmp
        
        Moment = cv2.moments(mask)
        if Moment['m00'] > 0:
            cx = int(Moment['m10']/Moment['m00'])
            cy = int(Moment['m01']/Moment['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            if err > 0:
                errtmp = err
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.vel_publisher.publish(self.twist)
        else:
            self.twist.linear.x = 0
            if errtmp <10:
            	errtmp=errtmp*10
            self.twist.angular.z = errtmp / 50
            self.vel_publisher.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

errtmp = 20
rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
