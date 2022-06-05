#!/usr/bin/env python
from re import A
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image


class Circling():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.flag = 1
        self.flag2 = 0
        self.err = 100
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber(
            "/spur/laser/scan", LaserScan, self.callback)
        self.image_sub = rospy.Subscriber('rm3/camera/image_raw',
                                          Image, self.image_callback)
        self.twist = Twist()

    def callback(self, msg):
        regions = {
            'right':  min(min(msg.ranges[1013:1014]), 10),
            'front':  min(min(msg.ranges[128:384]), 10),
            # 'left':   min(min(min(msg.ranges[0:12]), min(msg.ranges[955:1023])), 10),
            'rightFront':  min(min(msg.ranges[10:13]), 10),
            # 'leftFront':   min(min(min(msg.ranges[0:12]), min(msg.ranges[955:1023])), 10),
        }
        self.take_action(regions)

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image = image[0:800, 100:700]
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape

        Moment = cv2.moments(mask)
        if Moment['m00'] > 0:
            cx = int(Moment['m10']/Moment['m00'])
            cy = int(Moment['m01']/Moment['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            self.flag2 = 1
            self.err = cx - w/2
            if self.flag == 1:
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(self.err) / 1000
                self.vel_publisher.publish(self.twist)

        else:
            self.flag2 = 0
            if self.flag == 1:
                self.twist.linear.x = 0
                self.twist.angular.z = -float(self.err) / 1000
                self.vel_publisher.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def take_action(self, regions):
        threshold_dist = 0.9
        linear_speed = 0.5
        angular_speed = 0.8

        linear_x = 0
        angular_z = 0
        print(self.flag)
        state_description = ''
        if regions['front'] > threshold_dist:
            self.flag = 1

        if regions['front'] < threshold_dist:
            state_description = 'Front'
            print("test")
            linear_x = 0
            angular_z = angular_speed
            self.flag = 0

        if (abs(regions['rightFront'] - regions['right'])) > 0.00001 and regions['rightFront'] < regions['right'] and regions['rightFront'] < 1.5:
            state_description = 'rightFront less right'
            linear_x = linear_speed
            angular_z = -0.4
            if self.flag2==1:
            	self.flag = 1
            else:
            	self.flag = 0

        if (abs(regions['rightFront'] - regions['right'])) > 0.00001 and regions['rightFront'] > regions['right'] and regions['right'] < 1.5:
            state_description = 'rightFront more right'
            linear_x = linear_speed
            angular_z = 0.4
            if self.flag2==1:
            	self.flag=1
            else:
            	self.flag = 0

        if self.flag == 0:
            self.twist.linear.x = linear_x
            self.twist.angular.z = angular_z
            self.vel_publisher.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    Circling()
    errtmp = 20

    rospy.spin()
