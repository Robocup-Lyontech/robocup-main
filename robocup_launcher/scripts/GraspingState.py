#!/usr/bin/env python  

__author__ ='Raphael Leber'

import rospy 
import actionlib
from std_msgs.msg import String, Int16, Empty

from robocup_launcher.srv import GraspingStateSrv, GraspingStateSrvResponse
from control_msgs.msg import JointTrajectoryControllerState



class GraspingState():

    def __init__(self):
        rospy.init_node('grasping_state_node', anonymous=False)

        # --------------
        # Declare topics
        # --------------
        # * Suscribers
        rospy.Subscriber("/pmb2_gripper_controller/state", JointTrajectoryControllerState, self.gripper_state_callback)
        # * Publishers
        #self.pubPerson = rospy.Publisher('/message/person', String, queue_size=1)

        # --------------
        # Declare Services
        # --------------
        self.service = rospy.Service('grasping_state', GraspingStateSrv, self.serve_grasping_state)



        self.open_th = -0.599 * 2
        self.close_th = 0.299 * 2
        self.eps = 0.003
        self.gripper_state = "undefined" # "opened" "opening" "closing" "grasping" "closed"
        self.previous_gripper_gap = -100

        rospy.loginfo("[grasping_state_node] init")

        rospy.spin()        


    def gripper_state_callback(self, state):
        gripper_gap = state.desired.positions[0] + state.desired.positions[1] 

        if( gripper_gap >= self.close_th):
            self.gripper_state = "closed"
        elif(gripper_gap <= self.open_th):
            self.gripper_state = "opened"
        elif( self.previous_gripper_gap != -100):
            if( (self.previous_gripper_gap - gripper_gap) > self.eps ):
                self.gripper_state = "opening"
            elif( (self.previous_gripper_gap - gripper_gap) < -self.eps ):
                self.gripper_state = "closing"        
            else:     
                self.gripper_state = "grasping"     

        self.previous_gripper_gap = gripper_gap



    def serve_grasping_state(self, req):

        gs = GraspingStateSrvResponse()

        gs.grasping_state = self.gripper_state
        rospy.loginfo("[grasping_state_node] service called - answer : %s", gs.grasping_state)

        return gs


if __name__ == '__main__':
    gs = GraspingState()