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
from math import pi
from std_msgs.msg import String


class MoveitPregrasp:
    
    def __init__(self):
        self.init_ROS_and_moveit()
        self.get_basic_info()
        self.plan_pose_goal()
        # self.plan_cartesian_path()

    def init_ROS_and_moveit(self):
        ## init MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        ## init ROS
        rospy.init_node('moveit_pregrasp', anonymous=False)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    def plan_pose_goal(self):
        ## plan a motion for this group to a desired pose for the end-effector
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = -0.1523
        pose_goal.pose.position.y = -0.0336
        pose_goal.pose.position.z =  0.6566 - 0.1  
        
## FIXME: this part should be organized
#### for tf2_ros pose transform
        print("!!!!!!!!!!!!!!!!!!", pose_goal.header.frame_id)  # should be "" (nothing) for now

        import tf2_ros
        import tf2_geometry_msgs

        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform( "mdworld", # target frame
                                                "camera_frame", # source frame
                                                rospy.Time(0), # get the tf at first available time
                                                rospy.Duration(1.0)) #wait for 1 second
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
#### done

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