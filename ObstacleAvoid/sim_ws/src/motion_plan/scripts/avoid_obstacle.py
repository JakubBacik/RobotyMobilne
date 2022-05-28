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
        self.flag = 0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.sub = rospy.Subscriber("/spur/laser/scan", LaserScan, self.callback)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) 

    def callback(self, msg): 
        print('-------RECEIVING LIDAR SENSOR DATA-------')
        print('Front:  {}'.format(min(min(msg.ranges[640:896]), 10))) 
        print('RightFront: {}'.format(min(min(msg.ranges[522:523]), 10)))
        print('Right: {}'.format(min(min(msg.ranges[502:503]), 10)))
        print('Back: {}'.format(msg.ranges[256])) 
        regions = {
            'right':  min(min(msg.ranges[502:503]), 10),
            'front':  min(min(msg.ranges[640:896]), 10),
            #'left':   min(min(min(msg.ranges[0:12]), min(msg.ranges[955:1023])), 10),
            'rightFront':  min(min(msg.ranges[522:523]), 10),
            #'leftFront':   min(min(min(msg.ranges[0:12]), min(msg.ranges[955:1023])), 10),
        }
        self.take_action(regions)


    def odometry(self, msg): 
        print()#msg.pose.pose)

    def take_action(self,regions):
        threshold_dist = 0.9
        linear_speed = -0.2
        angular_speed = 0.8
        
        linear_x = 0
        angular_z = 0
        
        state_description = ''
        

 
        if regions['front'] > threshold_dist:
            state_description = 'No obstacle'
            linear_x = linear_speed
            angular_z = 0
        elif regions['front'] < threshold_dist:
            state_description = 'Front'
            linear_x = 0
            angular_z = angular_speed    
  

        if (abs(regions['rightFront'] - regions['right'])) > 0.00001 and regions['rightFront'] < regions['right'] and  regions['rightFront'] < 1.5:
            state_description = 'rightFront less right'
            linear_x = linear_speed
            angular_z = 0.2
        
        if (abs(regions['rightFront'] - regions['right'])) > 0.00001 and regions['rightFront'] > regions['right'] and regions['right'] < 1.5:
            state_description = 'rightFront more right'
            linear_x = linear_speed
            angular_z = -0.2
        

        rospy.loginfo(state_description)
        circle.linear.x = linear_x
        circle.angular.z = angular_z
        self.pub.publish(circle)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node') 
    Circling() 
    rospy.spin()   