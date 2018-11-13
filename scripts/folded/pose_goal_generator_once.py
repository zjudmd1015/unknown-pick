#!/usr/bin/env python

## working on virturalenv: unknown-pick_v2

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
#### Reference: https://github.com/atenpas/gpd 

import rospy
from gpd.msg import GraspConfigList
from geometry_msgs.msg import Pose

class PoseGoalGenerator:
    '''
    Select a grasp for the pose goal of robot's end effector.
    '''
    def __init__(self):
        self.grasps = []  # variable to store grasps
        self.FLAG_grasp_received = False

        self.init_ROS()
        self.utility()

    def init_ROS(self):
        rospy.init_node("get_grasps", anonymous = False)
        self.sub = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.callback)
        self.pub = rospy.Publisher("/get_grasps/pose_goal", Pose, queue_size=1)
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        '''
        Callback function to receive grasps.
        '''
        self.grasps = msg.grasps
        if self.FLAG_grasp_received == True:
            self.sub.unregister()

    def utility(self):
        while not rospy.is_shutdown():
            print(".")
            if len(self.grasps) > 0:
                rospy.loginfo('Received %d grasps.', len(self.grasps))
                self.FLAG_grasp_received = True
                break
            self.rate.sleep()
        grasp = self.grasps[0] # grasps are sorted in descending order by score
        print('Selected grasp with score:', grasp.score)
        pose_goal = self.grasp2pose(grasp)

        while not rospy.is_shutdown():
            self.pub.publish(pose_goal)
            self.rate.sleep()
    
    def grasp2pose(self, grasp):
        '''
        Trans GPD msg format to Pose format.
        '''
        pose = Pose()
        pose.position = grasp.bottom
        # pose.orientation.x = 0.0
        # pose.orientation.y = 0.0
        # pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose
        
        

def main():
    pg_gen = PoseGoalGenerator()

if __name__ == "__main__":
    main()