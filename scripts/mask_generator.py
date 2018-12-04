#!/usr/bin/env python

## working on virturalenv: mask_rcnn

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
#### Reference: https://github.com/matterport/Mask_RCNN

import os
import sys
#FIXME package incompatiblity
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
        self.init_model()
        self.init_ROS()

    def init_ROS(self):
        rospy.init_node("mask_generator", anonymous = False)
        self.service = rospy.Service('generate_mask', mask_req, self.handle_mask_req)
        self.bridge = CvBridge()
        try:
            rospy.loginfo("Mask Generator Service is ready ------")
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")


    def init_model(self):
        self.config = InferenceConfig()
        # config.display()

        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=self.config)
        # Load weights trained on MS-COCO
        self.model.load_weights(COCO_MODEL_PATH, by_name=True)
        self.model.keras_model._make_predict_function()   ## w/o this, there's some error

        # COCO Class names
        # Index of the class in the list is its ID. For example, to get ID of
        # the teddy bear class, use: class_names.index('teddy bear')
        self.class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                       'bus', 'train', 'truck', 'boat', 'traffic light',
                       'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                       'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
                       'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                       'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                       'kite', 'baseball bat', 'baseball glove', 'skateboard',
                       'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
                       'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                       'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                       'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
                       'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
                       'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
                       'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                       'teddy bear', 'hair drier', 'toothbrush']

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

        # plt.imshow(rgb_img)
        # plt.show()

        is_found, obj_mask_cv2 = self.detect(obj_name, rgb_img)
        obj_mask_ros = self.bridge.cv2_to_imgmsg(obj_mask_cv2, "mono8")

        print('------ ------    END     ------ ------')
        print()

        return [is_found, obj_mask_ros]

    def detect(self, obj_name, rgb_img):
        # from keras.backend import clear_session
        # Run detection

        results = self.model.detect([rgb_img], verbose=1)
        # clear_session()
        r = results[0]

        ## objects may not exist
        obj_id = self.class_names.index(obj_name)
        obj_idx = np.where(r['class_ids'] == obj_id)
        if len(obj_idx[0]) == 0:
            is_found = False
            obj_mask = np.zeros(rgb_img.shape[:2], dtype='uint8')
        else:
            print("---- INFO ----")
            print("scores: {}".format(r['scores'][obj_idx]))
            print("mask shape: {}".format(r['masks'].shape))

            is_found = True
            obj_idx = obj_idx[0][0]
            obj_mask = r['masks'][:,:,obj_idx]
            ## change bool value to uint8
            obj_mask = (obj_mask * 255).astype('uint8')

            # plt.imshow(obj_mask)
            # plt.show()

        return is_found, obj_mask

def main():
    mask_gen = MaskGenerator()

if __name__ == "__main__":
    main()
