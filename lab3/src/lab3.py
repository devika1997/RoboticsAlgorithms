#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def lab3():
   
   pub = rospy.Publisher('lab3_topic', String, queue_size=10)
   rospy.init_node('lab3', anonymous=True)
   rate = rospy.Rate(10)
   userinput = input("Please give Input:")
   while not rospy.is_shutdown():
       pub.publish(userinput)
       rate.sleep()

if __name__ == '__main__':
 try:
  lab3()
 except rospy.ROSInterruptException:
  pass
