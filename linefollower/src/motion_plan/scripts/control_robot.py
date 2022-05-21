#!/usr/bin/env python
from asyncio import current_task
from cmath import pi
from secrets import choice
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
import math
from math import atan2, sqrt

class ControlRobot():
    
    def __init__(self):
        rospy.init_node('rm3_robot_control', anonymous = True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pos_subscriber = rospy.Subscriber('/odom', Odometry, self.clbk_msg)

        # position of our robot
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # speed of our robot
        self.speed = 1
        self.angular_speed = 15.0*2*math.pi/360

        self.rate = rospy.Rate(5)
        

        
    def clbk_msg(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def euclidean_distance(self, position1, position2):
        return sqrt(pow((position2.x - position1.x), 2) +
                    pow((position2.y - position1.y), 2))

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        
    def move_in_straight_way(self):
        vel_msg = Twist()      
        actual_position = Point()
        start_position = Point()

        print("Move robot forward")
        distance = float(input("Distance: "))
        isForward = int(input("Direction(1 for forward, 0 for backward): "))

        if(isForward):
            vel_msg.linear.x = abs(self.speed)
        else:
            vel_msg.linear.x = -abs(self.speed)

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        start_position.x = actual_position.x = self.x
        start_position.y = actual_position.y = self.y

        while(self.euclidean_distance(start_position, actual_position) < distance):
            self.vel_publisher.publish(vel_msg)
            actual_position.x = self.x
            actual_position.y = self.y
        
        self.stop_robot() 
        print("Done")

    def rotate(self):
        vel_msg = Twist()

        print("Rotate robot")
        angle = int(input("Rotation angel: "))
        isClockwise = int(input("Direction(1 for right, 0 for left): "))

        relative_angel = angle*math.pi/180

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        if(isClockwise):
            vel_msg.angular.z = -abs(self.angular_speed)
        else:   
            vel_msg.angular.z = abs(self.angular_speed)
            
        t0 = rospy.Time.now().to_sec()
        currnet_angel = 0
        while(currnet_angel < relative_angel):
            self.vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            currnet_angel = self.angular_speed*0.85*(t1-t0)

        self.stop_robot()
        print("Done")

    def move2point(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        goal = Point()
        actual_position = Point()
        actual_position.x = self.x
        actual_position.y = self.y
        print("Move to point")
        goal.x = float(input("x: "))
        goal.y = float(input("y: "))

        angel_to_goal = atan2(self.y, self.x)
        while(self.euclidean_distance(goal, actual_position) >= 0.2):
            actual_position.x = self.x
            actual_position.y = self.y
            inc_x = goal.x - self.x
            inc_y = goal.y - self.y
            angel_to_goal = atan2(inc_y, inc_x)
            if math.fabs(angel_to_goal - self.yaw)>0.1:
                vel_msg.linear.x = 0
                vel_msg.angular.z = self.angular_speed
            else:
                vel_msg.linear.x = self.speed
                vel_msg.angular.z = 0

            self.vel_publisher.publish(vel_msg)
            
        self.stop_robot()
        print("Done x:{self.x} y:{self.y}")


if __name__ == '__main__':
    try:
        control = ControlRobot()
        choice = 0
        while(choice != 4):
            print("1 move in staright way")
            print("2 rotate")
            print("3 go to point")
            print("4 finish")
            choice = int(input("Your choice: "))
            if choice == 1:
                control.move_in_straight_way()
            elif choice == 2:
                control.rotate()
            elif choice == 3:
                control.move2point()
            elif choice == 4:
                exit()

        rospy.spin() 
    except rospy.ROSInterruptException: 
        pass
