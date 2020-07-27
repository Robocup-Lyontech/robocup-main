__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy
import actionlib
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import random
import copy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_color_detection_srvs.srv import DetectColorFromImg

from ros_people_mng_msgs.msg import PeopleMetaInfo,PeopleMetaInfoList
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg
from ros_openpose_gossip_srvs.srv import OpenPoseGossip

from FaceDetectionModule import FaceDetectionModule
from PersonMetaInfo import PersonMetaInfo


class DetectPeopleMeta():


    def __init__(self,is_face_bounding_box_used):
        self.is_face_bounding_box_used=is_face_bounding_box_used
        self.configure()

    def configure(self):
        ## wait for openpose joints detection
        try:
            rospy.wait_for_service('/people_pose_from_img',50)
            rospy.loginfo("service people_pose_from_img READY")
            self._openPoseSrv = rospy.ServiceProxy('people_pose_from_img', DetectPeoplePoseFromImg)
        except Exception as e:
            rospy.logwarn("Service people_pose_from_img call failed: %s" % e)
        ## wait for gossip pose detection
        try:
            rospy.wait_for_service('/openpose_gossip_srv',5)
            rospy.loginfo("service openpose_gossip_srv READY")
            self._gossipPoseSrv = rospy.ServiceProxy('openpose_gossip_srv', OpenPoseGossip)
        except Exception as e:
            rospy.logwarn("Service openpose_gossip_srv call failed: %s" % e)
        ## wait for color detection service
        try:
            rospy.wait_for_service('/detect_color_srv',5)
            rospy.loginfo("service detect_color_srv READY")
            self._colorDetectionSrv = rospy.ServiceProxy('detect_color_srv', DetectColorFromImg)
        except Exception as e:
            rospy.logwarn("Service detect_color_srv call failed: %s" % e)
        self._faceProcess=FaceDetectionModule()
        self._bridge = CvBridge()
        ## wait for gossip pose detection
        #try:
        #    rospy.wait_for_service('/<TODO>',5)
        #    rospy.loginfo("service <TODO> READY")
        #    self._openPoseSrv = rospy.ServiceProxy('<TODO>', DetectColorFromImg)
        #except Exception as e:
        #    rospy.logwarn("Service <TODO> call failed: %s" % e)

    def callOpenpose(self, img):
        try:
            resp1 = self._openPoseSrv(img)
            rospy.logdebug("-------------------  NB PEOPLE: "+str(len(resp1.personList.persons))+"-----------------------")
            #rospy.loginfo( "service:"+str(len(resp1.personList))
            return resp1.personList
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"+str(e))
            return None

    def callGossipPose(self, persons):
        try:
            resp1 = self._gossipPoseSrv(persons)
            #rospy.loginfo("Pose Gossip Of Service")
            #rospy.loginfo( "service:"+str(resp1))
            return resp1
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"+str(e))
            return None

    def callColorDetection(self, img):
        try:
            resp1 = self._colorDetectionSrv(img)
            #rospy.loginfo("Colors")
            #rospy.loginfo( "main:"+str(resp1.main_color))
            #rospy.loginfo( "colors:"+str(resp1.main_colors))
            return resp1
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"+str(e))
            return None

    def getPeopleInImg(self, img):
        """
        Call OpenPose to get peoples in the given image
        Ouput : openpose_ros_msgs/Persons persons
        """
        rospy.logdebug("------- Process Data: OPENPOSE -------")
        start_time = time.time()
        persons = self.callOpenpose(img)
        rospy.logdebug("---------------------------------------------------:OPENPOSE: timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")
        return persons

    def getPeopleGossip(self, persons):
        """
        Extract gossip from persons detected by OpenPose
        Ouput : ros_openpose_gossip_msgs/PersonsGossip persons_gossip
        """
        rospy.logdebug("------- Process Data: GOSSIPPOSE -------")
        start_time = time.time()
        persons_gossip = self.callGossipPose(persons)
        rospy.logdebug("---------------------------------------------------: GOSSIPPOSE: timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")
        return persons_gossip

    def getPersonClothsColor(self, person_gossip, cv_img, person_meta):
        """
        Extract the meta information from a person gossip
        """
        # Start color detection
        start_time = time.time()
        rospy.logdebug("------- Process Data: Main Color Detection -------")
        rospy.logdebug("COLOR DETECTION: PERSON:")
        rospy.logdebug(person_gossip)
        # Does he have a t-shirt ?
        if len(person_gossip.shirtRect.points) == 2 :
            # Crop the teeshirt
            person_meta.setBoundingBox(PersonMetaInfo.SHIRT_RECT, person_gossip.shirtRect.points)
            cv_img_crop = cv_img[
                int(person_gossip.shirtRect.points[0].y):int(person_gossip.shirtRect.points[1].y),
                int(person_gossip.shirtRect.points[0].x):int(person_gossip.shirtRect.points[1].x)]
            # Get t-shirt main color
            if cv_img_crop.size != 0:
                # Convert cv image to image msg to send to service
                img_crop = self._bridge.cv2_to_imgmsg(cv_img_crop, encoding="bgr8")
                dominant_color = self.callColorDetection(img_crop)
                person_meta.setMainColor(PersonMetaInfo.SHIRT_RECT, dominant_color.main_color.color_name, dominant_color.main_color.rgb)
                person_meta.setColorList(PersonMetaInfo.SHIRT_RECT, dominant_color.main_colors.colorList)
                rospy.logdebug("id:"+str(person_meta.id)+"-shirtRect-color:"+str(dominant_color.main_color.color_name))
            else:
                rospy.logwarn("T-Shirt Crop Img = []")
        # Does he have a trouser ?
        if len(person_gossip.trouserRect.points) == 2 :
            # Crop the trouser
            person_meta.setBoundingBox(PersonMetaInfo.TROUSER_RECT, person_gossip.trouserRect.points)
            cv_img_crop = cv_img[
                int(person_gossip.trouserRect.points[0].y):int(person_gossip.trouserRect.points[1].y),
                int(person_gossip.trouserRect.points[0].x):int(person_gossip.trouserRect.points[1].x)]
            # Get trouser main color
            if cv_img_crop.size !=0:
                img_crop = self._bridge.cv2_to_imgmsg(cv_img_crop, encoding="bgr8")
                dominant_color = self.callColorDetection(img_crop)
                person_meta.setMainColor(PersonMetaInfo.TROUSER_RECT,dominant_color.main_color.color_name,dominant_color.main_color.rgb)
                person_meta.setColorList(PersonMetaInfo.TROUSER_RECT,dominant_color.main_colors.colorList)
                rospy.logdebug("id:"+str(person_meta.id)+"-trouserRect-color:"+str(dominant_color.main_color.color_name))
            else:
                rospy.logwarn("Trouser Crop Img = []")
        rospy.logdebug("---------------------------------------------------: COLOR timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")

    def recognizePersonFace(self, person_gossip, cv_img, person_meta):
        # Start color face detection
        start_time = time.time()
        rospy.logdebug("------- Process Data: FACE DETECTION -------")
        rospy.logdebug("FACE DETECTION: BOUNDING BOX:")
        rospy.logdebug(person_gossip)
        # Choice of the target image
        is_img_face = False
        if self.is_face_bounding_box_used:
            target_box = person_gossip.headRect.points
            is_img_face = True
        else:
            target_box = person_gossip.boundingBox.points
        # Crop the image
        if len(target_box) !=0:
            cv_img_crop = cv_img[int(target_box[0].y):int(target_box[1].y), int(target_box[0].x):int(target_box[1].x)]
            img_crop = self._bridge.cv2_to_imgmsg(cv_img_crop, encoding="bgr8")
            label, score = self._faceProcess.detectFaceOnImg(img_crop, is_img_face)
            person_meta.label_id = str(label)
            person_meta.label_score = score
        else:
            rospy.logwarn("No head bounding box for person_gossip:"+str(person_gossip.id))
            person_meta.label_id = str('NoHead')
            person_meta.label_score = 0.0
        rospy.logdebug("---------------------------------------------------: FACE DETECTION timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")

    def learnPersonFace(self, person_gossip, cv_img, name):
        """
        Learn face attributes from person_gossip
        """
        start_time = time.time()
        rospy.logdebug("------- Process Data: FACE LEARNING -------")
        rospy.logdebug("FACE LEARNING: BOUNDING BOX:")
        rospy.logdebug(person_gossip)
        output = True
        # Find the head box
        # box = person_gossip.headRect.points
        box = person_gossip.boundingBox.points
        # If the head box is ok
        if len(box) !=0:
            # Crop the head from the image
            cv_img_crop = cv_img[int(box[0].y):int(box[1].y), int(box[0].x):int(box[1].x)]
            img_crop = self._bridge.cv2_to_imgmsg(cv_img_crop, encoding="bgr8")
            # Learn from it
            img_id = self._faceProcess.processFaceOnImg(img_crop, name)
            # Was it succesfull ?
            if img_id is None:
                rospy.logwarn("Failed to save face from person_gossip:"+str(person_gossip.id))
                output = False
        else:
            rospy.logwarn("No head bounding box for person_gossip:"+str(person_gossip.id))
            output = False
        rospy.logdebug("---------------------------------------------------: FACE LEARNING timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")
        return output

    def deletePersonsFacesDb(self):
        """
        """
        return self._faceProcess.deleteFacesFromDb()

    def firstEncounter(self, img, name):
        """
        First time meeting someone - Map his name and update the face database
        """
        # Get persons
        persons = self.getPeopleInImg(img)
        # If no persons here : stop
        if persons is None:
            rospy.logwarn("Failed to detect person with OpenPose")
            return None
        # If more than one person here : stop
        if len(persons.persons) > 1:
            rospy.logwarn("More than 1 person detected. Unable to learn name.")
            return None
        # We have persons - Time to get their gossip
        persons.image_w = img.width
        persons.image_h = img.height
        persons_gossip = self.getPeopleGossip(persons)
        # If no gossip here : stop
        if persons_gossip is None:
            rospy.logwarn("Failed to get gossip from person detected with OpenPose")
            return None
        # Get person gossip
        person_gossip = persons_gossip.personsGossip.personsGossip[0]
        # Convert image msg to cv img for crop purpose
        cv_img = self._bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        # Learn Face
        if self.learnPersonFace(person_gossip, cv_img, name) == False:

            return None
        # Create person Meta
        person_meta = PersonMetaInfo(person_gossip.id)
        person_meta.label_id = name
        person_meta.posture = person_gossip.posture
        person_meta.handPosture = person_gossip.handPosture
        person_meta.distanceEval = person_gossip.distanceEval
        person_meta.setBoundingBox(PersonMetaInfo.PERSON_RECT, person_gossip.boundingBox.points)
        person_meta.setPosition(person_gossip.pose)
        # Get clothes colors
        self.getPersonClothsColor(person_gossip, cv_img, person_meta)
        # Output
        return person_meta

    def recognizePeople(self, img):
        """
        Recognize people that have been already met
        """
        # Get persons
        persons = self.getPeopleInImg(img)
        # If no persons here : stop
        if persons is None:
            rospy.logwarn("Failed to detect persons with OpenPose")
            return None
        # We have persons - Time to get their gossip
        persons.image_w = img.width
        persons.image_h = img.height
        persons_gossip = self.getPeopleGossip(persons)
        # If no gossip here : stop
        if persons_gossip is None:
            rospy.logwarn("Failed to get gossip from person detected with OpenPose")
            return None
        # Convert image msg to cv img for crop purpose
        cv_img = self._bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        # Process everyone recognition
        personMetaInfoMap = {}
        for person_gossip in persons_gossip.personsGossip.personsGossip:
            # Create person Meta
            person_meta = PersonMetaInfo(person_gossip.id)

            #person_meta.label_id = person_gossip.name

            person_meta.posture = person_gossip.posture
            person_meta.handPosture = person_gossip.handPosture
            person_meta.distanceEval = person_gossip.distanceEval
            person_meta.setBoundingBox(PersonMetaInfo.PERSON_RECT, person_gossip.boundingBox.points)
            person_meta.setPosition(person_gossip.pose)
            # Label the face
            self.recognizePersonFace(person_gossip, cv_img, person_meta)
            # Get clothes colors
            self.getPersonClothsColor(person_gossip, cv_img, person_meta)
            # Copy the meta information in the global map
            personMetaInfoMap[person_meta.id]=copy.deepcopy(person_meta)
        #Output
        return personMetaInfoMap

    def getRandomPt(self,h,w):
        x=random.uniform(0, h)
        y=random.uniform(0, w)
        return x,y
