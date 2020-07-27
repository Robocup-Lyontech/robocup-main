#! /usr/bin/env python

from __future__ import print_function
import rospy

from robocup_msgs.msg import InterestPoint, Action, NavigationAction
from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg
from navigation_manager.msg import NavMngAction, NavMngGoal

from actionlib_msgs.msg import GoalStatus

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
# import actionlib_tutorials.msg
#import pmb2_nav_action.msg
import geometry_msgs


# Strategy
from common.navstrategy.SimplyGoNavStrategy import SimplyGoNavStrategy
from common.navstrategy.GoAndRetryNavStrategy import GoAndRetryNavStrategy
from common.navstrategy.GoCleanRetryNavStrategy import GoCleanRetryNavStrategy
from common.navstrategy.GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy
from common.navstrategy.GoCRRCloseToGoal import GoCRRCloseToGoal

class GoToPositionActionClient():
    client = None

    def __init__(self):
        try:
            rospy.init_node('pmb2_GoTo_action_client')
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
        
        rospy.loginfo("Connecting to navigation_manager action server ... ")
        self.client = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
        finished1 = self.client.wait_for_server(timeout = rospy.Duration(20.0))
        if finished1:
            rospy.loginfo("navigation_manager Connected")
        else:
            rospy.logwarn("Unable to connect to navigation_manager action server")
            return
        self.ChoosePose()

    def GoToPosition(self, point):
        # orderState0a=self.sendNavOrderAction("NP","CRRCloseToGoal", "DOOR_TO_ENTRANCE_02",120.0)
        # def sendNavOrderAction(self,action,mode,itP,timeout):
        try:
            action = "NP"
            # action = ""
            mode = "CRRCloseToGoal"
            # itP = "DOOR_TO_ENTRANCE_02"
            itP = ""
            timeout = 120.0

            goal = NavMngGoal()
            goal.action=action
            goal.itP=itP
            goal.navstrategy=mode
            
            # goal.itP_point.x = -3.09927248955
            # goal.itP_point.y = 0.147106766701
            goal.itP_point = point

            rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(timeout))
            state=self.client.get_state()
            # if state == GoalStatus.ABORTED:
            #     rospy.logwarn("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
            # else:
            #     rospy.loginfo("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
            # return state
        except Exception as e:
                rospy.logwarn("###### NAV ACTION FAILURE , State: %s",str(e))
        # return GoalStatus.ABORTED

        # Prints out the result of executing the action
        return self.client.get_result()
    
    def ChoosePose(self):
        while not rospy.is_shutdown():
            # Pose = PoseStamped()
            pointGoal = geometry_msgs.msg.Point()
            print
            print('Options : ')
            print(' - 0 : Quit')
            print(' - 1 : Entrance')
            print(' - 2 : Kitchen')
            print(' - 3 : Charging Place')
            answer = input('Where do you want to send the robot ? ')
            if(answer == 0):
                break
            elif(answer == 1): #Entrance
                # Pose.header.frame_id = 'map'
                pointGoal.x = -3.09927248955
                pointGoal.y = 0.147106766701
                # Pose.pose.orientation.z = 0.965981851705
                # Pose.pose.orientation.w = 0.25860986481
            elif (answer == 2): #kitchen
                # Pose.header.frame_id = 'map'
                pointGoal.x = -3.16048765182
                pointGoal.y = 2.15104579926
                # Pose.pose.orientation.z = 0.978578773184
                # Pose.pose.orientation.w = 0.205872739024
            elif(answer == 3): #charging place
                # Pose.header.frame_id = 'map'
                pointGoal.x = -0.257431030273
                pointGoal.y = -1.1128064394
                # Pose.pose.orientation.z = 0.581594191623
                # Pose.pose.orientation.w = 0.731730386103
            else:
                print('Wrong answer, cancelling')
            result = self.GoToPosition(pointGoal)
            

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pmb2_GoTo_action_client')
        result = GoToPositionActionClient()
        # print("Result: " + str(result.result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)