#!/usr/bin/env python

## working on virturalenv: unknown-pick_v2 (now no need of virtualenv)

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
#### Reference: https://github.com/atenpas/gpd 

import rospy
from gpd.msg import GraspConfigList
from geometry_msgs.msg import Pose

import numpy as np 
from pyquaternion import Quaternion

import tf2_ros
from geometry_msgs.msg import TransformStamped

class PoseGoalGenerator:
    '''
    Select a grasp for the pose goal of robot's end effector.
    '''
    def __init__(self):
        self.grasps = []  # variable to store grasps
        self.init_ROS()

    def init_ROS(self):
        ## ROS node initialization
        rospy.init_node("pose_goal_generator", anonymous = False)

        ## parameter setup
        self.is_pose_goal_fixed = rospy.get_param("~is_pose_goal_fixed")
        self.grasp_detection_topic = rospy.get_param("~grasp_detection_topic")

        ## utilities setup
        self.sub = rospy.Subscriber(self.grasp_detection_topic, GraspConfigList, self.callback)
        self.pub = rospy.Publisher("~pose_goal", Pose, queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()  # tf2_ros broadcaster
        self.tf = None # initially no tf, so do not broadcast
        self.br_rate = rospy.Rate(1000) # 1000 hz

        # try:
        #     rospy.spin()
        # except KeyboardInterrupt:
        #     print("Shutting down")
        while not rospy.is_shutdown():
            self.tf_broadcaster()
            self.br_rate.sleep()

    def callback(self, msg):
        '''
        Callback function to receive grasps.
        '''
        self.grasps = msg.grasps
        self.utility()

    def utility(self):
        if len(self.grasps) <= 0:
            print(".")
        else:
            rospy.loginfo('Received %d grasps.', len(self.grasps))

            grasp = self.grasps[0] # grasps are sorted in descending order by score
            print('Selected grasp with score:', grasp.score)
            pose_goal = self.grasp2pose(grasp)
            
            self.pub.publish(pose_goal)
            # self.rate.sleep()
    
    def grasp2pose(self, grasp):
        '''
        Trans GPD msg format to Pose format, meanwhile broadcast a tf of grasp_frame.
        '''
        ## Pose
        pose = Pose()
        if not self.is_pose_goal_fixed:
            ## for Panda Robot
            pose.position = grasp.bottom

            appr, bino, axis = grasp.approach, grasp.binormal, grasp.axis
            rx =        np.array([axis.x, axis.y, axis.z])
            ry = -1.0 * np.array([bino.x, bino.y, bino.z])
            rz =        np.array([appr.x, appr.y, appr.z])
            Rot = np.matrix([rx, ry, rz]).T
            quat = Quaternion(matrix = Rot)

            pose.orientation.w = quat[0]
            pose.orientation.x = quat[1]
            pose.orientation.y = quat[2]
            pose.orientation.z = quat[3]
        else:
            ## for debug
            pose.position.x = -0.1523
            pose.position.y = -0.0336
            pose.position.z =  0.6566 
            pose.orientation.w = 1.0

        ## tf 'grasp_frame'
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_frame"
        t.child_frame_id = "grasp_frame"
        t.transform.translation = pose.position
        t.transform.rotation = pose.orientation

        # FIXME: the frequency is too low for tf broadcaster
        # TODO: should change a broadcasting mechanism
        self.tf = t

        return pose

    def tf_broadcaster(self):
        if self.tf is not None:
            self.br.sendTransform(self.tf)
        

def main():
    pg_gen = PoseGoalGenerator()

if __name__ == "__main__":
    main()