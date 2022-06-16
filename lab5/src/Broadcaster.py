#!/usr/bin/env python
import roslib
import rospy
import nav_msgs.msg
import tf


class Broadcaster:
    def run(self,msg, robot):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                        (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                        rospy.Time.now(),
                        robot,
                        "world")

if __name__ == '__main__':
    rospy.init_node('broadcast')
    robot = rospy.get_param('~robot')
    b= Broadcaster()
    rospy.Subscriber('/%s/base_pose_ground_truth' % robot,
                     nav_msgs.msg.Odometry,
                     b.run,
                     robot)
    rospy.spin()
