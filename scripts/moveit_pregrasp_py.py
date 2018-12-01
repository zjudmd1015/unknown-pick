#!/usr/bin/env python

## working on virturalenv: N/A, but better using Python2.7 (compatible with MoveIt!)

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
#### Reference: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

import sys
import copy
import rospy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import tf2_ros
import tf2_geometry_msgs
        
from trajectory_msgs.msg import JointTrajectory

class MoveitPregrasp:
    
    def __init__(self):
        rospy.sleep(5.)
        self.init_ROS_and_moveit()

        ## for offline debug
        self.get_basic_info()
        # self.plan_pose_goal()
        # self.plan_cartesian_path()

        ## listen to /get_grasps/pose_goal to decide pose goal
        # self.open_gripper()
        self.follow_pose_goal()


    def follow_pose_goal(self):
        self.pose_goal_subscriber_ = rospy.Subscriber("/pose_goal_generator/pose_goal", geometry_msgs.msg.Pose, self.cb_follow)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def cb_follow(self, msg):
        
        # pose_cam = geometry_msgs.msg.PoseStamped()
        # pose_cam.pose.orientation = msg.orientation 
        # pose_cam.pose.position = msg.position
        # pose_goal.pose.position.z -= 0.3  # in camera_frame

        # FIXME: actually the Pose in the "grasp_frame" is just aligned with the frame, so no need to transform it (no need for "pose_cam")
        pose_grasp = geometry_msgs.msg.PoseStamped()
        pose_grasp.pose.orientation.w = 1.0
        pose_grasp.pose.position.z -= 0.2


        ## tf transform 
        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        ## transformation from camera_frame to panda_hand,
        ##            and from panda_hand to mdworld
        # try:
            # transform_cam_hand = tf_buffer.lookup_transform( 
            #                     "panda_hand", # target frame
            #                     "camera_frame", # source frame
            #                     rospy.Time(0), # get the tf at first available time
            #                     rospy.Duration(1.0)) # wait for 1 second
            # transform_hand_world = tf_buffer.lookup_transform(
            #                     "mdworld", # target frame
            #                     "panda_hand", # source frame
            #                     rospy.Time(0), # get the tf at first available time
            #                     rospy.Duration(1.0)) # wait for 1 second
            # transform_cam_grasp = tf_buffer.lookup_transform(
            #                     "camera_frame", # source frame
            #                     "grasp_frame", # target frame
            #                     rospy.Time(0), # get the tf at first available time
            #                     rospy.Duration(1.0)) # wait for 1 second
        transform_grasp_world = tf_buffer.lookup_transform(
                            "mdworld", # target frame
                            "grasp_frame", # source frame
                            rospy.Time(0), # get the tf at first available time
                            rospy.Duration(1.0)) # wait for 1 second
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("Some error with tf2_ros transformation.")
        
        # ## actual transformation
        # pose_hand = tf2_geometry_msgs.do_transform_pose(pose_cam, transform_cam_hand)
        # # FIXME: wrong! should follow grasp's z axis but not current hand z axis
        # pose_hand.pose.position.z -= 0.2 # in panda_hand frame
        # pose_mdworld = tf2_geometry_msgs.do_transform_pose(pose_hand, transform_hand_world)

        # TODO: finish the transformation code here
        pose_mdworld = tf2_geometry_msgs.do_transform_pose(pose_grasp, transform_grasp_world)

        ## plan and execute
        self.group.set_pose_target(pose_mdworld)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def open_gripper(self):
        self.group_gripper = moveit_commander.MoveGroupCommander("hand")

        gripper_goal = self.group_gripper.get_current_joint_values()
        gripper_goal[0] = 0.04
        gripper_goal[1] = 0.04

        self.group_gripper.go(gripper_goal, wait=False)
        self.group_gripper.stop()

    def close_gripper(self):
        pass

    def init_ROS_and_moveit(self):
        ## init MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # group_name = "arm" # for MICO
        group_name = "mico_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        ## init ROS
        rospy.init_node('moveit_pregrasp', anonymous=False)
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    def plan_pose_goal(self):
        ## plan a motion for this group to a desired pose for the end-effector
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.pose.position.x = -0.1523
        pose_goal.pose.position.y = -0.0336
        pose_goal.pose.position.z =  0.6566 - 0.3  # roughly as a pregrasp pose (here in camera_frame)

        ## Notice ##
        # 1. notice the order from pyquaternion.Quaternion and Pose msg is different
        # 2. notice that the EEF frame config might be different from different robots

        pose_goal.pose.orientation.w = 1.0
        
        pose_goal.pose.orientation.w =  0.74186651
        pose_goal.pose.orientation.x = -0.07337342
        pose_goal.pose.orientation.y =  0.45955304
        pose_goal.pose.orientation.z = -0.48276436

## FIXME: this part should be organized
#### for tf2_ros pose transform
        print("!!!!!!!!!!!!!!!!!!", pose_goal.header.frame_id)  # should be "" (nothing) for now

        import tf2_ros
        import tf2_geometry_msgs

        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            transform = tf_buffer.lookup_transform( "mdworld", # target frame
                                                    "camera_frame", # source frame
                                                    rospy.Time(0), # get the tf at first available time
                                                    rospy.Duration(1.0)) # wait for 1 second
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Some error with tf2_ros transformation.")
        
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
#### Done

        self.group.set_pose_target(pose_transformed)

        ## call the planner to compute the plan and execute it.
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def plan_cartesian_path(self):
        ## plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through
        waypoints = []
        scale = 1

        wpose = self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        ## display trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

        ## execute the planned path
        self.group.execute(plan, wait=True)


    def get_basic_info(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print ("============ Robot Groups: %s" % group_names)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (self.robot.get_current_state())
        print ("")


def main():
    mp_gen = MoveitPregrasp()

if __name__ == "__main__":
    main()