#!/usr/bin/env python3
import rospy
import tf
import random
import math
import copy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Perception:
    def __init__(self):
        rospy.init_node('percept')
        self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
        self.ranges1 = []
        self.ang_in = 0
        self.ang_min = 0
        rospy.Subscriber('base_scan', LaserScan, self.callback)


        #slope and intercept
        self.m=math.inf
        self.c=0

    def callback(self,msg):
        self.ranges1 = msg.ranges
        self.ang_in = msg.angle_increment
        self.ang_min = msg.angle_min
    
    def ransac(self):

        self.mrk=Marker()
        self.mrk.header.frame_id = "base_link"
        self.mrk.header.stamp = rospy.Time.now()
        self.mrk.type = Marker.LINE_LIST
        self.mrk.ns = "ransacmarker" 
        self.mrk.pose.orientation.w = 1.0
        self.mrk.lifetime = rospy.Duration(0.5)
        self.mrk.scale.x=0.15
        self.mrk.color.b=1.0
        self.mrk.color.a=1.0
        self.p=Point()
        self.points1=[]
        self.max=0
        self.inliers_fin=[]
        self.outliers_fin=[]
        for it,rg in enumerate(self.ranges1,1):
            if rg<3:
                self.p.x=rg*math.cos((it*self.ang_in)+self.ang_min)
                self.p.y=rg*math.sin((it*self.ang_in)+self.ang_min)
                self.p.z=0
                self.points1.append(copy.deepcopy(self.p))

        while len(self.points1)>=10:
            l=0
            self.max=0
            for i in range(100):
                inliers=[]
                outliers=[]
                [p1,p2]=random.sample(self.points1,2)
                if(math.fabs(p2.y-p1.y)>0):
                    self.m=(p2.y-p1.y)/(p2.x-p1.x)
    

                #y=mx+c
                self.c = p2.y-(self.m * p2.x)
                for point in self.points1:
                    dist= math.fabs((self.m*point.x)-point.y+self.c)/math.sqrt((self.m**2)+1)
                    if(dist<=0.25):
                        inliers.append(point)
                    else:
                        outliers.append(point)

                
                if(self.max<len(inliers)):
                    self.max=len(inliers)
                    self.inliers_fin=copy.deepcopy(inliers)
                    self.outliers_fin=copy.deepcopy(outliers)

            self.points1=copy.deepcopy(self.outliers_fin)
            s=sorted(self.inliers_fin , key=lambda k:k.x)
            res=s[::len(s)-1]
            self.mrk.points.extend(res)
                
    def run(self):
        r = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.ransac()
                self.pub.publish(self.mrk)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
	per = Perception()
	per.run()
