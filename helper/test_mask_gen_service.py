#!/usr/bin/env python

####################################
## test_mask_gen_service.py
#### Auther: Miaoding (m.dai AT u.northwestern.edu DOT com)
#### Dependencies: 
####   1. mask_rcnn (https://github.com/matterport/Mask_RCNN)
#### Function:
####   A testing script for mask_generator service.
#### Notes:
####   1. working on virturalenv 'mask_rcnn';
####################################

import sys

#FIXME package incompatiblity
sys.path.insert(1, "/home/dylan2/ws_catkin/install/lib/python3/dist-packages")
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

import rospy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from unknown_pick.srv import *

def main():
    bridge = CvBridge()
    bgr_img = cv2.imread('/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/sample3/0001_color.jpg')
    interested_obj = "apple"

    try:
        ros_image = bridge.cv2_to_imgmsg(bgr_img, "bgr8")
    except CvBridgeError as e:
        print(e)

    rospy.wait_for_service('generate_mask')
    try:
        req_obj_mask = rospy.ServiceProxy('generate_mask', mask_req)
        response = req_obj_mask(interested_obj, ros_image)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

    print(response.is_found)
    try:
        cv_image = bridge.imgmsg_to_cv2(response.obj_mask, "mono8")
    except CvBridgeError as e:
        print(e)

    print(cv_image.max())
    print(type(cv_image))
    print(cv_image.shape)
    print(cv_image.min())

    rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
    plt.figure()
    plt.subplot(121)
    plt.imshow(rgb_img)
    plt.subplot(122)
    plt.imshow(cv_image)
    plt.show()

if __name__ == "__main__":
    main()
