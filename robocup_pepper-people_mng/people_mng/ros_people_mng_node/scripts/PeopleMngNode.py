#!/usr/bin/env python
__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy
import actionlib
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from sensor_msgs.msg import Image
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_color_detection_srvs.srv import DetectColorFromImg

from ros_people_mng_msgs.msg import PeopleMetaInfoDetails, PeopleMetaInfo, PeopleMetaInfoList, PeopleMetaInfoListWithoutImg
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgResult
from ros_people_mng_actions.msg import LearnPeopleFromImgAction, LearnPeopleFromImgResult
from ros_people_mng_actions.msg import GetPeopleNameFromImgAction, GetPeopleNameFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg, TakePictureService, TakePictureServiceResponse

from process.DetectPeopleMeta import DetectPeopleMeta
from process.PersonMetaInfo import PersonMetaInfo
from process.DisplayMetaData import DisplayMetaData
from process.PeopleMetaSimilarity import PeopleMetaSimilarity
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose

class PeopleMngNode():

    def __init__(self):
        # Node configuration
        rospy.init_node('people_mng_node', anonymous=False)
        self.configure()
        # Saved people Map
        self.peopleMetaInfoMap = {}
        # Similarities processing
        self.peopleMetaSimilarity = PeopleMetaSimilarity()
        self.peopleMetaSimilarity.CURRENT_POSE_SCORE = self.peopleMetaSimilarity.PROCESS_POSE_NONE_SCORE #No pose
        self.peopleMetaSimilarity.WEIGHT_POSE_SCORE = 0
        # Subscribe to the image
        self.sub_rgb = rospy.Subscriber("/image", Image, self.rgb_callback, queue_size=1)
        self.pub_people_meta_info = rospy.Publisher("/people_meta_info", PeopleMetaInfoList, queue_size=1)
        self.pub_people_meta_info_img = rospy.Publisher("/people_meta_info_img", Image, queue_size=1)
        self.pub_people_meta_info_without_img = rospy.Publisher("/people_meta_info_only",PeopleMetaInfoListWithoutImg,queue_size=1)

        # Declare ros service
        self.detectPeopleMetaSrv = rospy.Service('detect_people_meta_srv', ProcessPeopleFromImg, self.detectPeopleMetaSrvCallback)
        self.resetPeopleMetaInfoMap = rospy.Service('reset_people_meta_info_map_srv', Trigger, self.resetPeopleMetaInfoMapSrvCallback)
        self.takePictureService = rospy.Service('take_picture_service',TakePictureService, self.takePictureServiceCallback)


        # Create action servers and start them
        self.actionServer_detect_people = actionlib.SimpleActionServer('detect_people_meta_action', ProcessPeopleFromImgAction, self.executePeopleMetaDetectionActionServer, False)
        self.actionServer_detect_people.start()
        self.actionServer_learn_people = actionlib.SimpleActionServer('learn_people_meta_action', LearnPeopleFromImgAction, self.executePeopleMetaLearningActionServer, False)
        self.actionServer_learn_people.start()
        self.actionServer_get_people_name = actionlib.SimpleActionServer('get_people_name_action', GetPeopleNameFromImgAction, self.executeGetPeopleNameActionServer, False)
        self.actionServer_get_people_name.start()


        self.current_img = None
        # ROS loop
        rospy.spin()

    def configure(self):
        self.is_continuously_detecting = rospy.get_param('/is_continuously_detecting', False)
        self.is_face_bounding_box_used = rospy.get_param('/is_face_bounding_box_used', False)
        rospy.loginfo("Param: is_face_bounding_box_used:"+str(self.is_face_bounding_box_used))
        self._bridge = CvBridge()
        self.detect_people_meta = DetectPeopleMeta(self.is_face_bounding_box_used)
        data_folder = rospy.get_param('imgtest_folder', '../data')
        self.displayMetaData = DisplayMetaData(data_folder+"/icon/", False, True, False)

    def rgb_callback(self, data):
        #FIXME need to protect to avoid concurrency?
        self.current_img = data
        if self.is_continuously_detecting == True:
            image_to_process = self.current_img
            start_time=time.time()
            rospy.logdebug("---------------------------------------------------: timeElasped since start:"+str(0)+"s")
            try:
                peopleMetaInfoListWithoutImg = PeopleMetaInfoListWithoutImg()
                peopleMetaInfoList = PeopleMetaInfoList()
                people_list=[]
                result=self.detect_people_meta.recognizePeople(image_to_process)
                for person in result.values():
                    rospy.logdebug('-')
                    rospy.logdebug(str(person))
                    current_peopleMeta = self.convertPersonMetaInfoToRosMsg(person)
                    people_list.append(current_peopleMeta)
                peopleMetaInfoList.peopleList = people_list
                peopleMetaInfoList.img=image_to_process
                peopleMetaInfoListWithoutImg.peopleList = people_list
                # publish metaData
                rospy.logdebug( "---------------------------------------------------: timeElasped since start:" + str(round(time.time()-start_time,3)) + "s")
                self.pub_people_meta_info.publish(peopleMetaInfoList)
                self.pub_people_meta_info_without_img.publish(peopleMetaInfoListWithoutImg)


                # compute display
                cv_image = self._bridge.imgmsg_to_cv2(image_to_process, desired_encoding="bgr8")
                cv_img_to_display = self.displayMetaData.displayResult(peopleMetaInfoList,cv_image)
                msg_img = self._bridge.cv2_to_imgmsg(cv_img_to_display, encoding="bgr8")
                # publish image with MetaData
                self.pub_people_meta_info_img.publish(msg_img)
            except Exception as e:
                rospy.logwarn("unable to find or launch function corresponding :, error:[%s]", str(e))

    def takePictureServiceCallback(self, req):
        cv_image = self._bridge.imgmsg_to_cv2(self.current_img, desired_encoding="bgr8")

        # new_width = 194
        # new_height = 259
        scale_percent = 100

        new_width = int(cv_image.shape[1] * scale_percent / 100)
        new_height = int(cv_image.shape[0] * scale_percent / 100)

        dsize = (new_width, new_height)
        output = cv2.resize(cv_image, dsize)
        cv2.imwrite(req.path, output)
        reponse = "Picture successfully saved at " + req.path

        return TakePictureServiceResponse(reponse)

    def detectPeopleMetaSrvCallback(self, req):
        image_to_process=''
        if len(req.img.goal.data) == 0:
            image_to_process = self.current_img
        else:
            image_to_process=req.img
        peopleMetaInfoList=PeopleMetaInfoList()
        people_list=[]
        result=self.detect_people_meta.recognizePeople(image_to_process)
        for person in result.values():
            #rospy.logwarn('-')
            #rospy.logwarn(str(person))
            current_peopleMeta=self.convertPersonMetaInfoToRosMsg(person)
            people_list.append(current_peopleMeta)
        peopleMetaInfoList.peopleList=people_list
        return peopleMetaInfoList

    def resetPeopleMetaInfoMapSrvCallback(self, req):
        self.peopleMetaInfoMap = {}
        if (self.detect_people_meta.deletePersonsFacesDb() == True):
            return True, "success"
        else:
            return False, "failed to erase faces database"

    def processImg(self, img):
        #TODO
        pass

    def executePeopleMetaDetectionActionServer(self, goal):
        # Check if any image as input
        image_to_process=''
        if len(goal.img.data) == 0:
            # If not we take the existing current image
            if self.current_img != None:
                image_to_process = self.current_img
            else:
                 rospy.logwarn("current_img is currently no set, no image to process")
                 self.actionServer_detect_people.set_aborted()
                 return
        else:
            image_to_process=goal.img
        #Init action outputs
        isActionSucceed=False
        action_result = ProcessPeopleFromImgResult()
        #Process the image to detect/recognize people
        try:
            peopleMetaInfoList = PeopleMetaInfoList()
            personMetaInfoMap = self.detect_people_meta.recognizePeople(image_to_process)
            #Process the detection output
            for personMetaInfo in personMetaInfoMap.values():
                rospy.logdebug('-')
                rospy.logdebug(str(personMetaInfo))
                action_result.peopleMetaList.peopleList.append(self.convertPersonMetaInfoToRosMsg(personMetaInfo))
                isActionSucceed=True
        #Action output
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self.actionServer_detect_people.set_succeeded(action_result)
        else:
            self.actionServer_detect_people.set_aborted()

    def executePeopleMetaLearningActionServer(self, goal):
        # Check if any image as input
        image_to_process = None
        if len(goal.img.data) == 0:
            # If not we take the existing current image
            if self.current_img != None:
                image_to_process = self.current_img
            else:
                 rospy.logwarn("current_img is currently no set, no image to process")
                 self.actionServer_learn_people.set_aborted()
                 return
        else:
            image_to_process = goal.img
        #Init action outputs
        isActionSucceed = False
        action_result = LearnPeopleFromImgResult()
        #Process the image to detect/recognize people
        try:
            personMetaInfo = self.detect_people_meta.firstEncounter(image_to_process, goal.name)
            if personMetaInfo is not None:
                #Prepare output
                peopleMetaInfo = self.convertPersonMetaInfoToRosMsg(personMetaInfo)
                action_result.peopleMetaInfo = peopleMetaInfo
                isActionSucceed=True
                #Save people meta info (ROS msg) in the map
                self.peopleMetaInfoMap[goal.name] = peopleMetaInfo
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(action_result), str(e))
        #Action output
        if isActionSucceed:
            self.actionServer_learn_people.set_succeeded(action_result)
        else:
            self.actionServer_learn_people.set_aborted()

    def executeGetPeopleNameActionServer(self, goal):
        """
        Try to get the persons name from an image
        """
        # Check if any image as input
        image_to_process = None
        if len(goal.img.data) == 0:
            # If not we take the existing current image
            if self.current_img != None:
                image_to_process = self.current_img
            else:
                rospy.logwarn("current_img is currently no set, no image to process")
                self.actionServer_get_people_name.set_aborted()
                return
        else:
            image_to_process = goal.img
        #Init action outputs
        isActionSucceed = False
        action_result = GetPeopleNameFromImgResult()
        action_result.peopleMetaList.img = image_to_process
        #Actions
        try:
            #Process the image to detect/recognize people
            personMetaInfoMap = self.detect_people_meta.recognizePeople(image_to_process)
            #Process similaritie with the already known people
            for personMetaInfo in personMetaInfoMap.values(): #Loop on people found in the image
                rospy.logdebug('-')
                rospy.logdebug(str(personMetaInfo))
                peopleMetaInfo = self.convertPersonMetaInfoToRosMsg(personMetaInfo)
                # max_score = -1.0
                # max_name = ""
                # for name in self.peopleMetaInfoMap.keys(): #Find correspondences with people name saved in the memory
                #     pose = Pose()
                #     score = self.peopleMetaSimilarity.evaluate_people(self.peopleMetaInfoMap[name], peopleMetaInfo, pose)[0]
                #     if score > max_score:
                #         max_score = score
                #         max_name = name
                # #Update outputs
                # if max_score > 0.5:
                #      action_result.peopleNames.append(max_name)
                # else:
                #      action_result.peopleNames.append("Unknown")
                # action_result.peopleNamesScore.append(max_score)
                action_result.peopleNames.append(peopleMetaInfo.label_id)
                action_result.peopleNamesScore.append(peopleMetaInfo.label_score)
                action_result.peopleMetaList.peopleList.append(peopleMetaInfo)
                isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(action_result), str(e))
        #Action output
        if isActionSucceed:
            self.actionServer_get_people_name.set_succeeded(action_result)
        else:
            self.actionServer_get_people_name.set_aborted()

    def convertPersonMetaInfoToRosMsg(self, people):
        rospy.logdebug(people)
        current_people=PeopleMetaInfo()
        current_people.id=str(people.id)
        current_people.label_id=people.label_id
        current_people.label_score=people.label_score
        current_people.handPosture=people.handPosture
        current_people.posture=people.posture
        current_people.distanceEval=people.distanceEval
        current_people.shirt_color_name=people.getMainColor(PersonMetaInfo.SHIRT_RECT)
        current_people.trouser_color_name=people.getMainColor(PersonMetaInfo.TROUSER_RECT)
        current_people_details=PeopleMetaInfoDetails()
        current_people_details.boundingBox.points=people.getBoundingBox(PersonMetaInfo.PERSON_RECT)
        current_people_details.shirtRect.points=people.getBoundingBox(PersonMetaInfo.SHIRT_RECT)
        current_people_details.shirtColorList=people.getColorList(PersonMetaInfo.SHIRT_RECT)
        current_people_details.trouserRect.points=people.getBoundingBox(PersonMetaInfo.TROUSER_RECT)
        current_people_details.trouserColorList=people.getColorList(PersonMetaInfo.TROUSER_RECT)
        current_people.details=current_people_details
        current_people.pose=people.pose
        return current_people


def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()
