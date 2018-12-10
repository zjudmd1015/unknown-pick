#!/usr/bin/env python

## working on virturalenv: N/A

#### Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)

import sys
import rospy
from unknown_pick.srv import seg_req, coor_req

class Coordinator:

    def __init__(self):
        self.num_trial = 5  # used in requesting "/cloud_segmentor/segment_cloud" service

        self.init_ROS()

    def init_ROS(self):
        rospy.init_node("unknown_pick_coordinator", anonymous = False)
        self.service = rospy.Service('coordinate', coor_req, self.handle_req)

        ## private params
        # self.interest_obj = rospy.get_param("~interested_object")

        ## ROS spin
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def handle_req(self, req):
        rospy.loginfo("Coordinating command: {}".format(req.command))
        cmd = req.command.split('_')
        if cmd[0] == "grasp":
            interested_obj = cmd[1]
        
            for i in range(self.num_trial):
                rospy.wait_for_service("/cloud_segmentor/segment_cloud", timeout=3.0)
                try:
                    client = rospy.ServiceProxy("/cloud_segmentor/segment_cloud", seg_req)
                    response = client(interested_obj)
                    if response.is_done == True:
                        return True
                except rospy.ServiceException as e:
                    rospy.logwarn("{} out of {} trials of 'segment_cloud' service call failed: {}".format(i+1, self.num_trial, e))
                if i == self.num_trial - 1:
                    rospy.logwarn("Maximum number of trials reached. Please call the service again.")
                
        else:
            return False

        return False

def main():
    coordinator = Coordinator()

if __name__ == "__main__":
    main()