#!/usr/bin/env python3
import rospy
import tf
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Evader:
    def __init__(self):
        self.i=random.choice([-1,1])
        self.obs_count=0
        self.msg = Twist()
        rospy.init_node('evader')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_scan', LaserScan, self.controller)
    
    def controller(self,data):
        self.obs_count = 0
        scan_data=min(data.ranges[100:260])
        if scan_data < 1:
            self.obs_count = 1
            self.msg.angular.z = 0			
            self.msg.linear.x = 0
        
        if self.obs_count==1:
            self.msg.linear.x = 0
            self.msg.angular.z =self.i
            
        else:   	
            self.msg.angular.z = 0			
            self.msg.linear.x = 2
            self.i=random.choice([-1,1])

    def run(self):
        r = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.pub.publish(self.msg)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
	eva = Evader()
	eva.run()
