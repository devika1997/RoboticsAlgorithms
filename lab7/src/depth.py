#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge,CvBridgeError
from matplotlib import pyplot as plt


class Depth:
    def __init__(self):
        rospy.init_node('depth')
        self.leftimage=Image()
        self.rightimage=Image()
        self.camera_info=CameraInfo()
        self.left_color=Image()
        self.f=2.8
        self.b=120
        self.pizel_d_size=3
        self.pc=PointCloud2()
        self.frame_index=rospy.get_param('~frameindex')
        self.pub = rospy.Publisher('depth_topic',Image, queue_size=10)
        self.pub1 = rospy.Publisher('pc_topic',PointCloud2, queue_size=10)
        rospy.Subscriber('zed/zed_node/left/image_rect_gray', Image, self.callback)
        rospy.Subscriber('zed/zed_node/right/image_rect_gray', Image, self.callback1)
        rospy.Subscriber('zed/zed_node/left/camera_info', CameraInfo, self.callback2)
        rospy.Subscriber('zed/zed_node/left/image_rect_color', Image, self.callback3)

    def callback(self,data):
        self.leftimage=data

    def callback1(self,data):
        self.rightimage=data
    
    def callback2(self,data):
        self.camera_info=data

    def callback3(self,data):
        self.left_color=data

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                bridge = CvBridge()
                left_img= bridge.imgmsg_to_cv2(self.leftimage, '8UC1')
                right_img= bridge.imgmsg_to_cv2(self.rightimage, '8UC1')
                left_c= bridge.imgmsg_to_cv2(self.left_color, '8UC4')
                stereo = cv2.StereoBM_create(numDisparities=32, blockSize=21)
                disparity = stereo.compute(left_img,right_img)
                disparity = disparity.astype(np.float32)
                disparity = (disparity/16.0)/32
                dpth=((self.b*self.f)/disparity).astype(np.uint8)
                depth_img=bridge.cv2_to_imgmsg(dpth.astype(np.uint8))
                self.pub.publish(depth_img)
                if(self.left_color.header.seq==self.frame_index):
                    self.pc.height=max(self.left_color.height,depth_img.height)
                    self.pc.width=max(self.left_color.width,depth_img.width)
                    cx=(self.pc.width-1)/2
                    cy=(self.pc.height-1)/2
                    fx=1/self.camera_info.K[0]
                    fy=1/self.camera_info.K[4]
                    c_idx=0
                    final=[]
                    for v in range(depth_img.height):
                        p=[255]*16
                        value_idx=0
                        for u in range(depth_img.width):
                            value_idx=+3
                            dth = int(dpth[v][u])
                            if dpth[v][u]== 0:
                                p[0]=dth
                                p[4]=dth
                                p[8]=dth
                    
                            else:
                                p[8]= int(dth)
                                p[0] = int(abs((u - cx) * p[8] / (2.8*self.left_color.width)))
                                p[7] = int(abs((v - cy) * p[8] / (2.8*self.left_color.height)))
                            #self.pc.points.extend([p])
                            p[12]=left_c[v][value_idx][0]
                            p[13]=left_c[v][value_idx+1][1]
                            p[14]=left_c[v][value_idx+2][2]
                            #if(len(p)>=row_step):
                        final.extend(p)
                    self.pc.header.frame_id="base_link"
                    self.pc.point_step=16
                    self.pc.row_step=20480
                    self.pc.data=final
                self.pub1.publish(self.pc)
            except CvBridgeError as e:
                print(e)
            r.sleep()

if __name__ == '__main__':
    d=Depth()
    d.run()
