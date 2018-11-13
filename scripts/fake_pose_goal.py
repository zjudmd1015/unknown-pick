#!/usr/bin/env python

## working on virturalenv: N/A

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)

import rospy
from geometry_msgs.msg import Pose

def main():
    ## ROS node setup
    rospy.init_node("fake_pose_goal_publisher", anonymous = False)
    pub = rospy.Publisher("/get_grasps/pose_goal", Pose, queue_size=1)
    r = rospy.Rate(1)

    ## setup a fake pose goal - relative to 'camera_frame'
    pose = Pose()
    pose.position.x = -0.1451
    pose.position.y = -0.0653
    pose.position.z =  0.6797
    pose.orientation.w = 1.0
    
    ## publishing loop
    while not rospy.is_shutdown():
        pub.publish(pose)
        r.sleep()

if __name__ == "__main__":
    main()