#!/usr/bin/env python

## working on virturalenv: mask_rcnn (Due to requiring python3 for cv_bridge)

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)

#FIXME dependencies
import sys

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

sys.path.insert(1, "/home/dylan2/ws_catkin/install/lib/python3/dist-packages")
from cv_bridge import CvBridge, CvBridgeError

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2 

def main():
    bgr_img = cv2.imread(
        "/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_color.jpg")
    depth_img = cv2.imread(
        "/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_depth.png", -1)

    ## ROS node setup
    rospy.init_node("fake_kinect2_publisher", anonymous = False)
    bgr_pub = rospy.Publisher("/kinect2/qhd/image_color_rect", Image, queue_size=1)
    depth_pub = rospy.Publisher("/kinect2/qhd/image_depth_rect", Image, queue_size=1)

    bridge = CvBridge()

    ## convert OpenCV images to ROS image messages
    try:
        ros_bgr_img = bridge.cv2_to_imgmsg(bgr_img, "bgr8")
        ros_depth_img = bridge.cv2_to_imgmsg(depth_img, "16UC1")
    except CvBridgeError as e:
        print(e)
    
    ## construct the header
    h = Header()
    h.frame_id = "camera_frame"

    ## publish images in a rate of 1 Hz
    r = rospy.Rate(1)
    # for i in range(20):    ## lasting 30 seconds
    while not rospy.is_shutdown():
        ## update header timestamp
        h.stamp = rospy.Time.now()
        ros_bgr_img.header = h 
        ros_depth_img.header = h
        ## publish image message
        bgr_pub.publish(ros_bgr_img)
        depth_pub.publish(ros_depth_img)
        ## sleep
        r.sleep()

if __name__ == "__main__":
    main()