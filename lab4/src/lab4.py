#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def callback(data):
   if(data.data==50400730):
   	print('True')
   else:
   	print('False')
def lab4():
   
   rospy.init_node('lab4', anonymous=True)
   rospy.Subscriber("lab4_topic",Int32, callback)
   rospy.spin()

if __name__ == '__main__':
 lab4()
