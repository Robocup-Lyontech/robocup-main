#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from  ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgGoal
import actionlib
import operator
from process.DisplayMetaData import DisplayMetaData




def LoadImgAndUseAction():
    rospy.init_node('people_mng_node_action_test', anonymous=False)
    _bridge = CvBridge()
    test_folder=rospy.get_param('imgtest_folder','../data')

    displayMETA=DisplayMetaData(test_folder+"/icon/",False,True,True)

    #Load Image
    #img_loaded1 = cv2.imread(test_folder+'/group-in-line.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/group-diff-position.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/couple.jpg')
    img_loaded1 = cv2.imread(test_folder+'/group-sit.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/imgMulti4.png')
    #img_loaded1 = cv2.imread(test_folder+'/large-people.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/large-people2.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/multiple_isolate2.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/multiple_isolated1.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/multiple_isolated3.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/imageFrontPepper5.png')
    #img_loaded1 = cv2.imread(test_folder+'/imageFrontPepper4.png')
    # img_loaded1 = cv2.imread(test_folder + '/bbt4-s.jpg') # NOT AVAILABLE IN REPOSITORY

    


    rospy.loginfo("File tested: "+str(test_folder+'/group-diff-position.jpg'))

    
    msg_im1 = _bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")   
#
    client = actionlib.SimpleActionClient('detect_people_meta_action', ProcessPeopleFromImgAction)
#DisplayMetaData
    client.wait_for_server()
#
    goal = ProcessPeopleFromImgGoal(img=msg_im1)
#
    client.send_goal(goal)
#
    client.wait_for_result()
#
    content_result=client.get_result()
    rospy.loginfo(content_result)

    displayMETA.displayResult(content_result.peopleMetaList,img_loaded1)


    goal2 = ProcessPeopleFromImgGoal()

    client.send_goal(goal2)

    client.wait_for_result()

    content_result2=client.get_result()
    rospy.loginfo(content_result2)
    

if __name__ == '__main__':
    try:
        LoadImgAndUseAction()
    except rospy.ROSInterruptException:
        pass