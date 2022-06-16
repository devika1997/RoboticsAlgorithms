#!/usr/bin/env python3
import rospy
import tf
import geometry_msgs.msg
import math


class Listen:
    def __init__(self):
        rospy.init_node('listen')

    def run(self):
        r = rospy.Rate(10)
        try:
            listener = tf.TransformListener()
            pub = rospy.Publisher('/robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
            listener.waitForTransform("/robot_1", "/robot_0", rospy.Time.now() , rospy.Duration(4.0))
            rospy.sleep(rospy.Duration(2.0))
            while not rospy.is_shutdown():
                try:
                    now = rospy.Time.now()
                    past = now - rospy.Duration(2)
                    listener.waitForTransform('/robot_1', '/robot_0',rospy.Time.now() ,rospy.Duration(4.0))
                    (trans, rot) = listener.lookupTransformFull('/robot_1',now,'/robot_0', past,"/world")

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                angular = 4 * math.atan2(trans[1], trans[0])
                linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                pub.publish(cmd)

                r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    l=Listen()
    l.run()
