#! /usr/bin/env python

from __future__ import print_function
import rospy

# other import from TakeOutTheGarbage2019v2Scenario
from abc import ABCMeta, abstractmethod
import uuid
import time
import random
import json
import time

# Import abstract methods
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioService import AbstractScenarioService
from AbstractScenario import AbstractScenario
from LocalManagerWrapper import LocalManagerWrapper

from robocup_msgs.msg import InterestPoint, Action, NavigationAction
from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg
from navigation_manager.msg import NavMngAction


# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
# import actionlib_tutorials.msg
import pmb2_nav_action.msg
import geometry_msgs

# Strategy
from common.navstrategy.SimplyGoNavStrategy import SimplyGoNavStrategy
from common.navstrategy.GoAndRetryNavStrategy import GoAndRetryNavStrategy
from common.navstrategy.GoCleanRetryNavStrategy import GoCleanRetryNavStrategy
from common.navstrategy.GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy
from common.navstrategy.GoCRRCloseToGoal import GoCRRCloseToGoal


def GoToPositionActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    # client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
    
    # client = actionlib.SimpleActionClient('GoToPose', pmb2_nav_action.msg.GoToInterestPointAction)
    
    '''
    #########################################################################################################
    
    TODO Peut etre plutot faire équivalent de sendActionOrderAction (à piquer dans GeneralManager.py)
    car la je viens plutot de refaire l'abstract

    #########################################################################################################
    '''

    orderState0a=self.sendNavOrderAction("NP","CRRCloseToGoal", "DOOR_TO_ENTRANCE_02",120.0)




    #client = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
    rospy.loginfo("Connecting to navigation_manager action server ... ")
    client = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
    finished1 = client.wait_for_server(timeout = rospy.Duration(20.0))
    if finished1:
        rospy.loginfo("navigation_manager Connected")
    else:
        rospy.logwarn("Unable to connect to navigation_manager action server")

    # Creates a goal to send to the action server.
    #goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
    selected_pose = geometry_msgs.msg.PoseStamped()
    
    # Entrance pose
    selected_pose.header.frame_id = "map"
    selected_pose.pose.position.x = -3.09927248955
    selected_pose.pose.position.y = 0.147106766701
    selected_pose.pose.orientation.z = 0.965981851705
    selected_pose.pose.orientation.w = 0.25860986481


    goal = pmb2_nav_action.msg.GoToInterestPointGoal(selected_pose)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pmb2_GoTo_action_client')
        result = GoToPositionActionClient()
        print("Result: " + str(result.result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)