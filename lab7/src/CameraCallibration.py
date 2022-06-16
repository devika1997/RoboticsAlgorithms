#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import glob

def CameraCallibration():
   rospy.init_node('cc', anonymous=True)
   CB = (9,6)
   opoints = []
   ipoints = []
   p = np.zeros((1, CB[0] * CB[1], 3), np.float32)
   p[0,:,:2] = np.mgrid[0:CB[0], 0:CB[1]].T.reshape(-1, 2)
   criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
   path = rospy.get_param('~imagepath')
   images = glob.glob(path+'/*')
   for image in images:
    img = cv2.cvtColor(cv2.imread(image),cv2.COLOR_BGR2GRAY)
    r, crs = cv2.findChessboardCorners(img, CB, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if r == True:
        opoints.append(p)
        crs1 = cv2.cornerSubPix(img, crs, (10,10),(-1,-1), criteria)
        
        ipoints.append(crs1)
   _,cm,_,_,_= cv2.calibrateCamera(opoints, ipoints, img.shape[::-1], None, None)
   print("Intrinsic matrix : \n",cm)
      

if __name__ == '__main__':
 try:
  CameraCallibration()
 except rospy.ROSInterruptException:
  pass
