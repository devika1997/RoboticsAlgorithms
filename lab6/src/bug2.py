#!/usr/bin/env python3
import rospy
import tf
import random
import math
import copy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry


'''
    GOAL-SEEK   --> 0
    WALL-FOLLOW --> 1
'''

class Bug2:
    def __init__(self):
        rospy.init_node('bug2')
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rleft=0
        self.rmid=7
        self.rright=0
        self.botposx=0
        self.botposy=0
        self.botposz=0
        self.botangle=0
        self.goalposx=rospy.get_param('~goalx')
        self.goalposy=rospy.get_param('~goaly')
        self.goalposz=0
        self.cmdvel=Twist()
        self.cstate=0
        self.aangle=0
        self.oldposx=0
        self.oldposy=0
        rospy.Subscriber("/base_scan", LaserScan,self.callback)
        rospy.Subscriber("/odom", Odometry, self.callback1)
        rospy.sleep(1)
        #print("posx",self.botposx)
        #print("posy",self.botposy)
        self.m= (self.goalposy-self.botposy)/(self.goalposx-self.botposx)
        self.c= self.botposy - (self.m*self.botposx)

    def callback(self,data):
        self.rleft=data.ranges[360]
        self.rmid=min(data.ranges[120:240])
        self.rright=data.ranges[0]

    def callback1(self,msg):
        self.botposx=msg.pose.pose.position.x
        self.botposy=msg.pose.pose.position.y
        self.botposz=msg.pose.pose.position.z
        (r,p,self.botangle) = tf.transformations.euler_from_quaternion ([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
    
    def bug(self):
        self.cmdvel=Twist()
        if(self.cstate==0):
            if(self.rmid<0.5):
                self.cstate=1
                self.oldposx=copy.deepcopy(self.botposx)
                self.oldpoxy=copy.deepcopy(self.botposy)
            else:
                aangle=math.atan2(self.goalposy-self.botposy,self.goalposx-self.botposx)
                if aangle < 0:
                    aangle +=  2*math.pi
                if self.botangle < 0:
                   self.botangle +=  2*math.pi
                self.cmdvel.linear.x=2
                self.cmdvel.angular.z=aangle-self.botangle

        elif(self.cstate==1):
            dist = abs(self.m * self.botposx -self.botposy + self.c)/(math.sqrt(self.m**2 + 1))
            d=math.sqrt((self.botposy - self.oldposy)**2 + (self.botposx - self.oldposx)**2)
            if(self.rmid<0.5):
                self.cmdvel.angular.z=-1
            elif(self.rleft>0.6):
                self.cmdvel.angular.z=1
                self.cmdvel.linear.x=0.2
            elif(self.rleft<0.3):
                self.cmdvel.angular.z=-1
                self.cmdvel.linear.x=0.2
            else:
                if(dist<0.1 and d>0.2):
                    self.cstate=0
                self.cmdvel.linear.x=2



    def run(self):
        r = rospy.Rate(10)
        try:
            
            while not rospy.is_shutdown():
                d=math.sqrt((self.goalposy - self.botposy)**2 + (self.goalposx - self.botposx)**2)
                if(d>0.75):
                    self.bug()
                    self.pub.publish(self.cmdvel)
                    r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
	b = Bug2()
	b.run()
