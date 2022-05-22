#!/usr/bin/env python
from re import A
import rospy 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Circling(): 
   
    def __init__(self): 
        global circle
        circle = Twist()  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.sub = rospy.Subscriber("/spur/laser/scan", LaserScan, self.callback)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) 

    def callback(self, msg): 
        print('-------RECEIVING LIDAR SENSOR DATA-------')
        print('Front:  {}'.format(msg.ranges[748])) 
        print('Left: {}'.format(msg.ranges[768]))
        print('Right: {}'.format(msg.ranges[728]))
        print('Back: {}'.format(msg.ranges[256])) 
        regions = {
            'right':  min(min(msg.ranges[500:580]), 10),
            'front':  min(min(msg.ranges[581:956]), 10),
            'left':   min(min(min(msg.ranges[0:12]), min(msg.ranges[955:1023])), 10),
        }
        self.take_action(regions)


    def odometry(self, msg): 
        print()#msg.pose.pose)

    def take_action(self,regions):
        threshold_dist = 0.9
        linear_speed = -1.2
        angular_speed = -0.2
        
        linear_x = 0
        angular_z = 0
        
        state_description = ''
        
        if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
            state_description = 'No obstacle'
            linear_x = linear_speed
            angular_z = 0
        elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
            state_description = 'Front and left and right'
            linear_x = -linear_speed
            angular_z = angular_speed 
        elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
            state_description = 'Front'
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
            state_description = 'Right'
            linear_x = 0
            angular_z = -angular_speed
        elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
            state_description = 'Left'
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
            state_description = 'Front and right'
            linear_x = 0
            angular_z = -angular_speed
        elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
            state_description = 'Front and left'
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
            state_description = 'Left and right'
            linear_x = linear_speed
            angular_z = 0
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

        rospy.loginfo(state_description)
        circle.linear.x = linear_x
        circle.angular.z = angular_z
        self.pub.publish(circle)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node') 
    Circling() 
    rospy.spin()   