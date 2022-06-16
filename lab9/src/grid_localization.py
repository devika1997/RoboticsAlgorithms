#!/usr/bin/env python3
import rospy
import tf
import random
import numpy as np
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GridLocalization:
    def __init__(self):
        self.beliefy=np.zeros((20,1))
        self.beliefy[10][0]=1
        self.beliefx=np.zeros((1,20))
        self.beliefx[0][10]=1
        rospy.init_node('grid')
        self.command=''
        self.comm_dir=''
        self.comm_steps=0
        self.currpos=(10,10)
        self.motion=np.array([0.2,0.6,0.2])
        self.obs=np.array([0.1,0.2,0.4,0.2,0.1])
        self.pred=['L','R','U','D'] 
        self.observation=['X','Y']
        rospy.Subscriber('robot', String , self.callback)
        self.xy=np.zeros((20,20))
        self.x=np.zeros((1,20))
        self.y=np.zeros((20,1))
    
    def callback(self,msg):
        self.command=msg.data
        self.comm_dir=self.command[0]
        if(len(self.command)>=2):
            if(len(self.command)==3):
                self.comm_steps=int(self.command[1]+self.command[2])
            elif(len(self.command)==2):
                self.comm_steps=int(self.command[1])
            elif(self.comm_steps>19 or len(self.command)>3):
                print("Invalid Command")
            #print(self.comm_dir,self.comm_steps)
            if(self.comm_dir in self.pred):
                    self.prediction()
            elif(self.comm_dir in self.observation):
                self.observations()
            else:
                print("Invalid Command")
        else:
            print("Invalid Command")
   
    def prob(self,belief):
        pos=0
        count=0
        #print(belief)
        prob=np.zeros((1,20))
        if(self.comm_dir=='R' or self.comm_dir=='L'):
            pos=self.currpos[1]
        else:
            pos=self.currpos[0]
            #belief=belief.reshape((1,20))
            #print(belief.shape)
        #print(pos) 
        #print("before",self.currpos)
        if(self.comm_dir=='R' or self.comm_dir=='D'):
            if(pos+3<=19 and pos>=1):
                prob[0][pos-1:pos+2]=belief[0][pos-1]*np.array([0.2,0.6,0.2])
                prob[0][pos:pos+3]=prob[0][pos:pos+3]+belief[0][pos]*np.array([0.2,0.6,0.2])
                prob[0][pos+1:pos+4]=prob[0][pos+1:pos+4]+belief[0][pos+1]*np.array([0.2,0.6,0.2])
                count=1
            elif(pos==17):
                prob[0][pos-1:pos+2]=belief[0][pos-1]*np.array([0.2,0.6,0.2])
                prob[0][pos:pos+3]=prob[0][pos:pos+3]+belief[0][pos]*np.array([0.2,0.6,0.2])
                prob[0][-2:]=prob[0][-2:]+belief[0][pos+1]*np.array([0.2,0.8])
                count=1
            elif(pos==18):
                prob[0][pos-1:pos+2]=belief[0][pos-1]*np.array([0.2,0.6,0.2])
                prob[0][-2:]=prob[0][-2:]+belief[0][pos]*np.array([0.2,0.8])
                prob[0][-1:]=prob[0][-1:]+belief[0][pos+1]*np.array([1])
                count=1
            elif(pos==0):
                prob[0][0:2]=belief[0][pos]*np.array([0.8,0.2])
                prob[0][pos+1:pos+4]=prob[0][pos+1:pos+4]+belief[0][pos+1]*np.array([0.2,0.6,0.2])
                count=1
            if(self.comm_dir=='R' and count==1):
                self.currpos=(self.currpos[0],self.currpos[1]+1)
                self.beliefx=prob
                self.beliefx=self.beliefx/np.sum(self.beliefx)
            elif(self.comm_dir=='D' and count==1):
                self.currpos=(self.currpos[0]+1,self.currpos[1])
                self.beliefy=prob.reshape((20,1))
                self.beliefy=self.beliefy/np.sum(self.beliefy)
        else:
            
            if(pos-3>=0 and pos<=18):
                #print(belief)
                prob[0][pos-1:pos+2]=belief[0][pos+1]*np.array([0.2,0.6,0.2])
                prob[0][pos-2:pos+1]=prob[0][pos-2:pos+1]+belief[0][pos]*np.array([0.2,0.6,0.2])
                prob[0][pos-3:pos]=prob[0][pos-3:pos]+belief[0][pos-1]*np.array([0.2,0.6,0.2])
                count=1
            elif(pos==2):
                prob[0][pos-1:pos+2]=belief[0][pos+1]*np.array([0.2,0.6,0.2])
                prob[0][pos-2:pos+1]=prob[0][pos-2:pos+1]+belief[0][pos]*np.array([0.2,0.6,0.2])
                prob[0][0:2]=prob[0][0:2]+belief[0][pos-1]*np.array([0.8,0.2])
                count=1
            elif(pos==1):
                prob[0][pos-1:pos+2]=belief[0][pos+1]*np.array([0.2,0.6,0.2])
                prob[0][0:2]=prob[0][0:2]+belief[0][pos]*np.array([0.8,0.2])
                prob[0][0:1]=prob[0][0:1]+belief[0][pos-1]*np.array([1])
                count=1
            elif(pos==19):
                prob[0][pos-1:pos+1]=belief[0][pos]*np.array([0.2,0.8])
                prob[0][pos-3:pos]=prob[0][pos-3:pos]+belief[0][pos-1]*np.array([0.2,0.6,0.2])
                count=1
            if(self.comm_dir=='L' and count==1):
                self.currpos=(self.currpos[0],self.currpos[1]-1)
                self.beliefx=prob
                self.beliefx=self.beliefx/np.sum(self.beliefx)
            elif(self.comm_dir=='U' and count==1):
                self.currpos=(self.currpos[0]-1,self.currpos[1])
                self.beliefy=prob.reshape((20,1))
                self.beliefy=self.beliefy/np.sum(self.beliefy)
        #print("after",self.currpos)
        #print("X",self.beliefx)
        #print("Y",self.beliefy)
    
    def prediction(self):
        if(self.comm_dir=='R' or self.comm_dir=='L'):
            for i in range(self.comm_steps+1):
                self.prob(self.beliefx)   
        else:
            for i in range(self.comm_steps+1):
                self.prob(self.beliefy.reshape((1,20)))
            
    def observations(self):
        count=0
        if(self.comm_dir=='X'):
            update=np.ones((1,20))*0.05
            if(self.comm_steps+2<=19 and self.comm_steps-2>=0):
                update[0][self.comm_steps-2:self.comm_steps+3]=self.obs
                count=1
            elif(self.comm_steps==18):
                update[0][self.comm_steps-2:self.comm_steps+2]=self.obs[0:4]
                count=1
            elif(self.comm_steps==19):
                update[0][self.comm_steps-2:self.comm_steps+1]=self.obs[0:3]
                count=1
            elif(self.comm_steps==1):
                update[0][self.comm_steps-1:self.comm_steps+3]=self.obs[-4:]
                count=1
            elif(self.comm_steps==0):
                update[0][self.comm_steps:self.comm_steps+3]=self.obs[-3:]
                count=1
            if(count==1):
                self.beliefx=np.multiply(update,self.beliefx)
                self.beliefx=self.beliefx/np.sum(self.beliefx)
        else:
            update=np.ones((20,1))*0.05
            if(self.comm_steps+2<=19 and self.comm_steps-2>=0):
                update[self.comm_steps-2:self.comm_steps+3]=self.obs.reshape((-1,1))
                count=1
            elif(self.comm_steps==18):
                update[self.comm_steps-2:self.comm_steps+2]=self.obs[0:4].reshape((-1,1))
                count=1
            elif(self.comm_steps==19):
                update[self.comm_steps-2:self.comm_steps+1]=self.obs[0:3].reshape((-1,1))
                count=1
            elif(self.comm_steps==1):
                update[self.comm_steps-1:self.comm_steps+3]=self.obs[-4:].reshape((-1,1))
                count=1
            elif(self.comm_steps==0):
                update[self.comm_steps:self.comm_steps+3]=self.obs[-3:].reshape((-1,1))
                count=1
            if(count==1):
                self.beliefy=np.multiply(update,self.beliefy)
                #print(update.shape)
                self.beliefy=self.beliefy/np.sum(self.beliefy)

        
    def update(self,i):
        temp=np.flip(self.beliefy)
        self.xy.set_array(temp*self.beliefx)

    def updatex(self,j):
        self.x.set_array(self.beliefx)

    def updatey(self,k):
        self.y.set_array(np.flip(self.beliefy))
        

if __name__ == '__main__':
    grid = GridLocalization()
    temp=np.flip(grid.beliefy)
    
    fig1=plt.figure(figsize=(20,1))
    grid.x=plt.imshow((grid.beliefx),cmap=plt.cm.get_cmap('Blues').reversed(),origin='lower')
    a1=FuncAnimation(fig=fig1,func=grid.updatex)
    plt.xticks(np.arange(0,20, 1.0))
    plt.colorbar()

    fig2=plt.figure(figsize=(1,20))
    grid.y=plt.imshow((temp),cmap=plt.cm.get_cmap('Blues').reversed(),origin='lower')
    a2=FuncAnimation(fig=fig2,func=grid.updatey)
    plt.yticks(np.arange(0,20, 1.0))
    plt.colorbar()

    fig=plt.figure(figsize=(20,20))
    grid.xy=plt.imshow((temp*grid.beliefx),cmap=plt.cm.get_cmap('Blues').reversed(),origin='lower')
    a=FuncAnimation(fig=fig,func=grid.update)

    plt.xticks(np.arange(0,20, 1.0))
    plt.yticks(np.arange(0,20, 1.0))
    plt.colorbar()
    
    
    
    plt.show()
    rospy.spin()
