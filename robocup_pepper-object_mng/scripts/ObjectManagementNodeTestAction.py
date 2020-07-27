#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from  object_management.msg import ObjectDetectionAction, ObjectDetectionGoal
from object_management.msg import LookAtObjectAction, LookAtObjectGoal, LookAtObjectResult
import actionlib


def UseAction():
    rospy.init_node('object_management_node_action_test', anonymous=False)

    client = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)

    client.wait_for_server()
    labels=[]
    goal = ObjectDetectionGoal()
    goal.labels=labels

    client.send_goal(goal)

    client.wait_for_result()

    content_result=client.get_result()

    result=content_result.labelList

    #rospy.loginfo("color:%s, color_web:%s, color_temp:%s, color_brightness_name:%s, RGB:[%s,%s,%s], percentage:%s",str(result.color_name),str(result.color_web),str(result.color_temp),str(result.color_brightness_name),str(result.rgb[0]),str(result.rgb[1]),str(result.rgb[2]),str(result.percentage_of_img))
    #rospy.loginfo("ALL MAIN COLORS")
    rospy.loginfo(result)

#TODO :
def UseActionLookAtObject():
    rospy.init_node('object_management_node_action_test', anonymous=False)

    client = actionlib.SimpleActionClient('look_at_object_action', LookAtObjectAction)

    client.wait_for_server()
    labels=['person']
    goal = LookAtObjectGoal()
    goal.labels=labels
    goal.index=0
    goal.head=False
    goal.base=False
    goal.finger=2

    client.send_goal(goal)

    client.wait_for_result()

    content_result=client.get_result()

    result=content_result.nb_label

    #rospy.loginfo("color:%s, color_web:%s, color_temp:%s, color_brightness_name:%s, RGB:[%s,%s,%s], percentage:%s",str(result.color_name),str(result.color_web),str(result.color_temp),str(result.color_brightness_name),str(result.rgb[0]),str(result.rgb[1]),str(result.rgb[2]),str(result.percentage_of_img))
    #rospy.loginfo("ALL MAIN COLORS")
    rospy.loginfo(result)

#diningtable
#person
#sofa
#tvmonitor
#desperados
#jus_de_pomme
#oasis_tropical
#orangina
#redbull

    #labels=['person','chair']
    #goal2 = LookAtObjectGoal()
    #goal2.labels=labels
    #client.send_goal(goal2)

    #client.wait_for_result()
    #content_result=client.get_result()
    #result=content_result.labelList
    #rospy.loginfo(result)


if __name__ == '__main__':
    try:
        #UseAction()
        UseActionLookAtObject()
    except rospy.ROSInterruptException:
        pass
