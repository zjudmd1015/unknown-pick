#!/usr/bin/env python

## working on virturalenv: mask_rcnn

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
#### Reference: https://github.com/matterport/Mask_RCNN

import os
import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt

# Root directory of the project - Mask RCNN
ROOT_DIR = os.path.abspath("/home/dylan2/libraries/Mask_RCNN/")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
# Import COCO config
sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
import coco
# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")
# Local path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")
# Download COCO trained weights from Releases if needed
if not os.path.exists(COCO_MODEL_PATH):
    utils.download_trained_weights(COCO_MODEL_PATH)

## ROS related preparation
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import cv2
from sensor_msgs.msg import Image
sys.path.insert(1, "/home/dylan2/ws_catkin/install/lib/python3/dist-packages")
from cv_bridge import CvBridge, CvBridgeError
from unknown_pick.srv import *

class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

class MaskGenerator:

    def __init__(self):
        self.init_ROS()

    def init_ROS(self):
        rospy.init_node("fake_mask_generator", anonymous = False)
        self.service = rospy.Service('generate_mask', mask_req, self.handle_mask_req)
        self.bridge = CvBridge()
        try:
            print("Service is ready ------")
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def handle_mask_req(self, req):

        obj_name = req.interested_object

        print()
        print('------ ------ NEW REQUEST ------ ------')
        print('interested_object: ', obj_name)
        print(type(obj_name))

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(req.rgb_img, "rgb8")
        except CvBridgeError as e:
            print(e)

#FIXME the file can also be passed from local file, but not ROS message
        is_found = True
        obj_mask_cv2 = cv2.imread('/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/sample2/mask_255.jpg', -1)
        obj_mask_ros = self.bridge.cv2_to_imgmsg(obj_mask_cv2, "mono8")

        print('------ ------    END     ------ ------')
        print()

        return [is_found, obj_mask_ros]




def main():
    mask_gen = MaskGenerator()

if __name__ == "__main__":
    main()
