#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from dialogue_hri_actions.msg import DialogueSendSignalAction,DialogueSendSignalGoal,AddInMemoryAction,AddInMemoryGoal
import actionlib


def askForAction():
    rospy.init_node('dialogue_hri_node_action_test', anonymous=False)
    #
    client = actionlib.SimpleActionClient('dialogue_hri_signal', DialogueSendSignalAction)
#
    client.wait_for_server()
#
    #goal1 = DialogueSendSignalGoal()
    #goal1.signal_to_emit="Cocktail/ScenarioStart"
    #goal1.signal_to_wait=""
    #client.send_goal(goal1)
    #client.wait_for_result()
##
##
    #content_result=client.get_result()
    #state=client.get_state()
    #rospy.logwarn("###### DIALOGUE ACTION END , State: %s",str(state))
    #rospy.loginfo(content_result)
#
    #rospy.sleep(30.)
#
    goal2 = DialogueSendSignalGoal()
    goal2.signal_to_emit="Cocktail/OrdersStart"
    goal2.signal_to_wait="Cocktail/OrdersFinish"
    client.send_goal(goal2)
#
    client.wait_for_result(rospy.Duration.from_sec(30.0))
    state=client.get_state()
    content_result=client.get_result()
    ##WORK Need action server subscriber on /<action_server_name>/cancel
    client.cancel_all_goals()
    rospy.logwarn(content_result)
#
    #if state ==4:
    #    rospy.logwarn("###### DIALOGUE ACTION END , State: %s",str(state))
    #else:
    #    rospy.loginfo("###### DIALOGUE ACTION END , State: %s",str(state))
#
#


    #clientAddInMem = actionlib.SimpleActionClient('add_in_memory_action', AddInMemoryAction)
#
    #clientAddInMem.wait_for_server()
#
    #goal1 = AddInMemoryGoal()
    #goal1.memory_location="Robocup/Data/CocktailParty/Objects"
    #goal1.payload="{['orange','coca','water']}"
    #clientAddInMem.send_goal(goal1)
    #clientAddInMem.wait_for_result(rospy.Duration.from_sec(5.0))
    #state=clientAddInMem.get_state()
    #content_result=clientAddInMem.get_result()
    #if state ==4:
    #    rospy.logwarn("###### Add In Memory ACTION END , State: %s",str(state))
    #else:
    #    rospy.loginfo("###### Add In Memory ACTION END , State: %s",str(state))
#
#
    #rospy.loginfo(content_result)



if __name__ == '__main__':
    try:
        askForAction()
    except rospy.ROSInterruptException:
        pass