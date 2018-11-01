#!/usr/bin/env python

import rospy
from gpd.msg import GraspConfigList

# global variable to store grasps
grasps = []


# Callback function to receive grasps.
def callback(msg):
    global grasps
    grasps = msg.grasps


# ==================== MAIN ====================
# Create a ROS node.
rospy.init_node('get_grasps')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    print '.'
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break
    rate.sleep()
