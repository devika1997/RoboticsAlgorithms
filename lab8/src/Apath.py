#!/usr/bin/env python3
import rospy
import math
import tf
import copy
import numpy as np
import matplotlib as plt
from matplotlib import pyplot
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Node:

    def __init__(self,par=None,curr_pos=None):

        self.par=par
        self.curr_pos = curr_pos
        self.f = 0
        self.g = 0
        self.h = 0

class APath:
    def __init__(self):
        rospy.init_node('Astar')
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.callback)
        self.wstartposx=-8.0
        self.wstartposy=-2.0
        self.wgoalposx=rospy.get_param('~goalx')
        self.wgoalposy=rospy.get_param('~goaly')
        self.starty=int((18/2)+self.wstartposx)
        self.startx=int((20/2)-self.wstartposy)
        self.goaly=int((18/2)+self.wgoalposx)
        self.goalx=int((20/2)-self.wgoalposy)
        self.cmdvel=Twist()
        self.epsilon=1
        self.botangle=0
        self.neighbours=[(-1,0),(1,0),(0,-1),(0,1)]
        self.map = np.array([0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]).reshape((20,18))
        self.r=20
        self.c=18
        self.path=[]

    def callback(self,msg):
        self.wstartposx=msg.pose.pose.position.x
        self.wstartposy=msg.pose.pose.position.y
        self.wstartposz=msg.pose.pose.position.z
        (_,_,self.botangle) = tf.transformations.euler_from_quaternion ([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    def dis(self):
        #print(self.path)
        for i in range(self.r):
            for j in range(self.c):
                if self.map[i][j]==1:
                    print("1    ",end='')
                elif (i,j) in self.path:
                    print('*    ',end='')
                else:
                    print("     ",end='')
            print("\n")
        
        #matplotlib
        temp=self.map
        for i in self.path:
            temp[i[0]][i[1]]=2

        grid = pyplot.imshow(temp)
        pyplot.show(block=False)
        pyplot.pause(0.3)
        #pyplot.close()

    def convert(self):
        temp=[]
        for pos in self.path:
            x=int(pos[1]-(18/2))
            y=int((20/2)-pos[0])
            temp.append((x+0.5,y-0.5))
        return temp
    
    def controller(self):
        points=self.convert()
        tmp_path=points
        r = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.cmdvel=Twist()
                if(len(tmp_path)>0):
                    (x,y)=tmp_path[0]
                    #print(x,y)
                    r.sleep()
                    aangle=math.atan2(y-self.wstartposy,x-self.wstartposx)
                    ang=aangle-self.botangle
                    if ang < 0:
                        ang +=  2*math.pi
                    elif ang >math.pi*2:
                        ang -=  2*math.pi
                    #print(ang)
                    if(ang>0.4):
                        if (ang>math.pi):
                            self.cmdvel.angular.z=-1
                            self.cmdvel.linear.x=0
                            self.pub.publish(self.cmdvel)
                        elif(ang<math.pi):
                            self.cmdvel.angular.z=1
                            self.cmdvel.linear.x=0
                            self.pub.publish(self.cmdvel)
                    else:
                        dist=math.sqrt((y-self.wstartposy)**2+(x-self.wstartposx)**2)
                        #print(dist)
                        if(dist>0.25):
                            #print(dist)
                            self.cmdvel.linear.x=5
                            self.pub.publish(self.cmdvel)
                        if(dist<0.25):
                            tmp_path.pop(0)
                            #print("hi")
                else:
                    break
        except rospy.ROSInterruptException:
            pass
            
            
    def run(self):
        visited=[]
        not_visited=[]
        start=Node(None,(self.startx,self.starty))
        end=Node(None,(self.goalx,self.goaly))
        not_visited.append(start)
        temp=self.map

        while len(not_visited)>0:
            curr=not_visited[0]
            curr_i=0
            for i,val in enumerate(not_visited):
                if val.f<curr.f:
                    curr=val
                    curr_i=i
           
            del not_visited[curr_i]
            visited.append(curr)

            if(curr.curr_pos==end.curr_pos):
                temp = curr
                while temp is not None:
                    self.path.append(temp.curr_pos)
                    temp = temp.par
                #print(self.path)
                self.path.reverse()
                break

            n_nodes=[]
            for n in self.neighbours:
                n_pos=(curr.curr_pos[0]+n[0],curr.curr_pos[1]+n[1])
                if(n_pos[0]>self.r-1 or n_pos[1]>self.c-1 or n_pos[0]<0 or n_pos[1] <0):
                    continue
                if(self.map[n_pos[0]][n_pos[1]]!=0):
                    continue
                n=Node(curr,n_pos)
                n_nodes.append(n)

            for neighbour in n_nodes:
                count=0
                for val in visited:
                    if(val.curr_pos==neighbour.curr_pos):
                        count=1
                if(count==1):
                    continue

                neighbour.h=(neighbour.curr_pos[0]-end.curr_pos[0])**2+abs(neighbour.curr_pos[1]-end.curr_pos[1])**2
                #print(neighbour.h)
                neighbour.g=curr.g + 1
                neighbour.f = neighbour.g + (self.epsilon*neighbour.h)

                count1=0
                for val1 in not_visited:
                    if(val1.curr_pos==neighbour.curr_pos and val1.g<neighbour.g):
                        count1=1
                
                if(count1==1):
                    continue
                    
                not_visited.append(neighbour)


if __name__ == '__main__':
	a = APath()
	a.run()
	a.dis()
	a.controller()
