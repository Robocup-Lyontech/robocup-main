__author__ = 'Jacques Saraydaryan'

import rospy
import uuid
import actionlib
import time
import json
from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from dialogue_hri_actions.msg import DialogueSendSignalAction, DialogueSendSignalGoal, AddInMemoryAction, AddInMemoryGoal
from object_management.msg import ObjectDetectionAction, ObjectDetectionGoal
from object_management.msg import LookAtObjectAction, LookAtObjectGoal, LookAtObjectResult
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgGoal
from ros_people_mng_actions.msg import LearnPeopleFromImgAction, LearnPeopleFromImgGoal
from ros_people_mng_actions.msg import GetPeopleNameFromImgAction, GetPeopleNameFromImgGoal
# from dialogue_hri_actions.msg import RequestToLocalManagerGoal, RequestToLocalManagerAction

from actionlib_msgs.msg import GoalStatus

import cv2
from cv_bridge import CvBridge


class AbstractScenarioAction:
    _actionNavMng_server=None
    _actionTtsHri_server=None
    _enableNavAction=True
    _enableTtsAction=True
    _enableDialogueAction=True
    _enableAddInMemoryAction=True
    _enableObjectDetectionMngAction=False
    _enableLookAtObjectMngAction=False
    _enableLearnPeopleMetaAction=False
    _enableMultiplePeopleDetectionAction=False
    _enableGetPeopleNameAction=False
    _configurationReady=False
    _enableRequestToLocalManagerAction=False

    def __init__(self,config):
        self._bridge = CvBridge()
        pass

    def configure_intern(self):
        try:
            # if self._enableNavAction:
            #     rospy.loginfo("Connecting to navigation_manager action server ... ")
            #     self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
            #     finished1 =self._actionNavMng_server.wait_for_server(timeout = rospy.Duration(20.0))
            #     if finished1:
            #         rospy.loginfo("navigation_manager Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to navigation_manager action server")

            # if self._enableTtsAction:
            #     rospy.loginfo("Connecting to tts_hri action server ... ")
            #     self._actionTtsHri_server = actionlib.SimpleActionClient('tts_hri', TtsHriAction)
            #     finished2 = self._actionTtsHri_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished2:
            #         rospy.loginfo("tts_hri Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to tts_hri action server")

            # if self._enableDialogueAction:
            #     self._actionDialogueHri_server = actionlib.SimpleActionClient('dialogue_hri_signal', DialogueSendSignalAction)
            #     finished3 = self._actionDialogueHri_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished3:
            #         rospy.loginfo("dialogue_hri Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to dialogue_hri action server")

            # if self._enableAddInMemoryAction:
            #     self._actionAddInMemoryHri_server = actionlib.SimpleActionClient('add_in_memory_action', AddInMemoryAction)
            #     finished4 = self._actionAddInMemoryHri_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished4:
            #         rospy.loginfo("AddInMemoryHri Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to AddInMemoryHri action server")


            # if self._enableObjectDetectionMngAction:
            #     self._actionObjectDetectionMng_server = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)
            #     finished5 = self._actionObjectDetectionMng_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished5:
            #         rospy.loginfo("ObjectDetectionMng Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to ObjectDetectionMng action server")

            # if self._enableLookAtObjectMngAction:
            #     self._actionLookAtObjectMng_server = actionlib.SimpleActionClient('look_at_object_action', LookAtObjectAction)
            #     finished6 = self._actionLookAtObjectMng_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished6:
            #         rospy.loginfo("LookAtObjectMng Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to LookAtObjectMng action server")


            # if self._enableLearnPeopleMetaAction:
            #     self._actionLearnPeopleMeta_server = actionlib.SimpleActionClient('learn_people_meta_action', LearnPeopleFromImgAction)
            #     finished7 = self._actionLearnPeopleMeta_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished7:
            #         rospy.loginfo("learnPeopleMeta action server Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to learnPeopleMeta action server")
            #
            # if self._enableMultiplePeopleDetectionAction:
            #     self._actionMultiplePeopleDetection_server = actionlib.SimpleActionClient('detect_people_meta_action', ProcessPeopleFromImgAction)
            #     finished8 = self._actionMultiplePeopleDetection_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished8:
            #         rospy.loginfo("MultiplePeopleDetection Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to MultiplePeopleDetection action server")
            #
            # if self._enableGetPeopleNameAction:
            #     self._actionGetPeopleName_server = actionlib.SimpleActionClient('get_people_name_action', GetPeopleNameFromImgAction)
            #     finished9 = self._actionGetPeopleName_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished9:
            #         rospy.loginfo("GetPeopleName Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to GetPeopleName action server")

            # if self._enableRequestToLocalManagerAction:
            #     self._actionRequestToLocalManager_server = actionlib.SimpleActionClient('request_to_local_manager', RequestToLocalManagerAction)
            #     finished10 = self._actionGetPeopleName_server.wait_for_server(timeout = rospy.Duration(10.0))
            #     if finished10:
            #         rospy.loginfo("RequestToLocalManager Connected")
            #     else:
            #         rospy.logwarn("Unable to connect to RequestToLocalManager action server")

        except Exception as e:
            rospy.loginfo("Unable to connect to action server: %s" % e)
        self._configurationReady=True

    # def action_status_to_string(self, action_status_int):
    #     if action_status_int == GoalStatus.PENDING:
    #         return "PENDING"
    #     elif action_status_int == GoalStatus.ACTIVE:
    #         return "ACTIVE"
    #     elif action_status_int == GoalStatus.PREEMPTED:
    #         return "PREEMPTED"
    #     elif action_status_int == GoalStatus.SUCCEEDED:
    #         return "SUCCEEDED"
    #     elif action_status_int == GoalStatus.ABORTED:
    #         return "ABORTED"
    #     elif action_status_int == GoalStatus.REJECTED:
    #         return "REJECTED"
    #     elif action_status_int == GoalStatus.PREEMPTING:
    #         return "PREEMPTING"
    #     elif action_status_int == GoalStatus.RECALLING:
    #         return "RECALLING"
    #     elif action_status_int == GoalStatus.RECALLED:
    #         return "RECALLED"
    #     elif action_status_int == GoalStatus.LOST:
    #         return "LOST"

    # def sendNavOrderAction(self,action,mode,itP,timeout):
    #     try:
    #         goal = NavMngGoal()
    #         goal.action=action
    #         goal.itP=itP
    #         goal.navstrategy=mode
    #         rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
    #         self._actionNavMng_server.send_goal(goal)
    #         self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionNavMng_server.get_state()
    #         if state == GoalStatus.ABORTED:
    #             rospy.logwarn("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
    #         else:
    #             rospy.loginfo("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
    #         return state
    #     except Exception as e:
    #          rospy.logwarn("###### NAV ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED


    # def sendNavOrderActionToPt(self,action,mode,x,y,timeout):
    #     try:
    #         goal = NavMngGoal()
    #         goal.action=action
    #         goal.itP=''
    #         goal.itP_point.x=x
    #         goal.itP_point.y=y
    #         goal.navstrategy=mode
    #         rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
    #         self._actionNavMng_server.send_goal(goal)
    #         self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionNavMng_server.get_state()
    #         if state == GoalStatus.ABORTED:
    #             rospy.logwarn("###### NAV ACTION END , State: %s",self.action_status_to_string(state))
    #         else:
    #             rospy.loginfo("###### NAV ACTION END , State: %s", self.action_status_to_string(state))
    #         return state
    #
    #     except Exception as e:
    #          rospy.logwarn("###### NAV ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED
    #
    # def sendTtsOrderAction(self,action,txt,mode,lang,timeout):
    #     try:
    #         goalTts = TtsHriGoal()
    #         goalTts.action=action
    #         goalTts.txt=txt
    #         goalTts.mode=mode
    #         goalTts.lang=lang
    #         rospy.loginfo("### TTS ACTION PENDING : %s",str(goalTts).replace('\n',', '))
    #         self._actionTtsHri_server.send_goal(goalTts)
    #         self._actionTtsHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionTtsHri_server.get_state()
    #         rospy.loginfo("###### TTS ACTION END , State: %s",str(state))
    #         return state
    #     except Exception as e:
    #          rospy.logwarn("###### TTS ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED
    #
    #
    # def sendDialogueOrderAction(self,signal_to_emit,signal_to_wait,timeout):
    #     try:
    #         goalDialogue = DialogueSendSignalGoal()
    #         # signal send to naoqi
    #         goalDialogue.signal_to_emit=signal_to_emit
    #         # signal on wich action server will wait answer
    #         goalDialogue.signal_to_wait=signal_to_wait
    #         rospy.loginfo("### DIALOGUE ACTION PENDING : %s",str(goalDialogue).replace('\n',', '))
    #         # send the current goal to the action server
    #         self._actionDialogueHri_server.send_goal(goalDialogue)
    #         # wait action server result
    #         finished_before_timeout=self._actionDialogueHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionDialogueHri_server.get_state()
    #         result=self._actionDialogueHri_server.get_result()
    #         rospy.loginfo("###### DIALOGUE ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionDialogueHri_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state,result
    #     except Exception as e:
    #          rospy.logwarn("###### TTS ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None
    #
    # def addInPepperMemory(self,memory_location,json_payload,timeout):
    #     goalAddInMemory = AddInMemoryGoal()
    #     goalAddInMemory.memory_location=memory_location
    #     goalAddInMemory.payload=json_payload
    #     rospy.loginfo("### ADD IN MEMORY ACTION PENDING : %s",str(goalAddInMemory).replace('\n',', '))
    #
    #     # send the current goal to the action server
    #     self._actionAddInMemoryHri_server.send_goal(goalAddInMemory)
    #     # wait action server result
    #     finished_before_timeout=self._actionAddInMemoryHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #     state=self._actionAddInMemoryHri_server.get_state()
    #     result=self._actionAddInMemoryHri_server.get_result()
    #     rospy.loginfo("###### ADD IN MEMORY ACTION END , State: %s",str(state))
    #     # if timeout cancel all goals on the action server
    #     if finished_before_timeout:
    #         self._actionAddInMemoryHri_server.cancel_all_goals()
    #     # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #     return state,result


    # def getObjectInFrontRobot(self,labels,move_head,timeout):
    #     try:
    #         goalObjDetection = ObjectDetectionGoal()
    #         goalObjDetection.labels=labels
    #         goalObjDetection.moveHead=move_head
    #
    #         rospy.loginfo("### OBJECT DETECTION MNG GET OBJECT ACTION PENDING : %s",str(goalObjDetection).replace('\n',', '))
    #
    #         # send the current goal to the action server
    #         self._actionObjectDetectionMng_server.send_goal(goalObjDetection)
    #         # wait action server result
    #         finished_before_timeout=self._actionObjectDetectionMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionObjectDetectionMng_server.get_state()
    #         result=self._actionObjectDetectionMng_server.get_result()
    #         rospy.loginfo("###### OBJECT DETECTION MNG GET OBJECT ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionObjectDetectionMng_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state,result
    #     except Exception as e:
    #          rospy.logwarn("###### OBJECT DETECTION MNG ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None

    # def lookAtObject(self,labels,index,head,base,finger,timeout):
    #     try:
    #         goalLookAtObj = LookAtObjectGoal()
    #         goalLookAtObj.labels = labels
    #         goalLookAtObj.index = index
    #         goalLookAtObj.head = head
    #         goalLookAtObj.base = base
    #         goalLookAtObj.finger = finger
    #
    #         rospy.loginfo("### LOOK AT OBJECT MNG GET OBJECT ACTION PENDING : %s",str(goalLookAtObj).replace('\n',', '))
    #
    #         # send the current goal to the action server
    #         self._actionLookAtObjectMng_server.send_goal(goalLookAtObj)
    #         # wait action server result
    #         finished_before_timeout=self._actionLookAtObjectMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionLookAtObjectMng_server.get_state()
    #         result=self._actionLookAtObjectMng_server.get_result()
    #         rospy.loginfo("###### LOOK AT OBJECT MNG GET OBJECT ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionLookAtObjectMng_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state,result
    #     except Exception as e:
    #          rospy.logwarn("###### LOOK AT OBJECT MNG ACTION FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None

    # def learnPeopleMetaFromImgTopic(self, name, timeout):
    #     """ Appel de l'apprentissage des attributs d'une personne """
    #     goalLearnPeople = LearnPeopleFromImgGoal(name=name)
    #     state, result = self.learnPeopleMeta(goalLearnPeople, timeout)
    #     return state, result
    #
    # def learnPeopleMetaFromImgPath(self, img_path, name, timeout):
    #     """ Appel de l'apprentissage des attributs d'une personne """
    #     img_loaded = cv2.imread(img_path)
    #     msg_img = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
    #     goalLearnPeople = LearnPeopleFromImgGoal(name=name, img=msg_img)
    #     state, result = self.learnPeopleMeta(goalLearnPeople, timeout)
    #     return state, result
    #
    # def learnPeopleMeta(self, goalLearnPeople, timeout):
    #     try:
    #         rospy.loginfo("### LEARN PEOPLE ATTRIBUTES ACTION PENDING")
    #         # send the current goal to the action server
    #         self._actionLearnPeopleMeta_server.send_goal(goalLearnPeople)
    #         # wait action server result
    #         finished_before_timeout = self._actionLearnPeopleMeta_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state = self._actionLearnPeopleMeta_server.get_state()
    #         result = self._actionLearnPeopleMeta_server.get_result()
    #         rospy.loginfo("###### LEARN PEOPLE ATTRIBUTES ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionLearnPeopleMeta_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state, result
    #     except Exception as e:
    #          rospy.logwarn("###### LEARN PEOPLE ATTRIBUTES FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None
    #
    # def detectMetaPeopleFromImgTopic(self, timeout):
    #     goalMetaPeople = ProcessPeopleFromImgGoal()
    #     state, result = self.detectMetaPeople(goalMetaPeople, timeout)
    #     return state, result
    #
    # def detectMetaPeopleFromImgPath(self, img_path, timeout):
    #     img_loaded1 = cv2.imread(img_path)
    #     msg_im1 = self._bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")
    #     goalMetaPeople = ProcessPeopleFromImgGoal(img=msg_im1)
    #     state, result = self.detectMetaPeople(goalMetaPeople, timout)
    #     return state, result
    #
    # def detectMetaPeople(self, goalMetaPeople, timeout):
    #     try:
    #         rospy.loginfo("### DETECT META PEOPLE ACTION PENDING")
    #         # send the current goal to the action server
    #         self._actionMultiplePeopleDetection_server.send_goal(goalMetaPeople)
    #         # wait action server result
    #         finished_before_timeout=self._actionMultiplePeopleDetection_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state=self._actionMultiplePeopleDetection_server.get_state()
    #         result=self._actionMultiplePeopleDetection_server.get_result()
    #         rospy.loginfo("###### DETECT META PEOPLE ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionMultiplePeopleDetection_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state,result
    #     except Exception as e:
    #          rospy.logwarn("###### DETECT META PEOPLE FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None
    #
    # def getPeopleNameFromImgTopic(self, timeout):
    #     goalPeopleName = GetPeopleNameFromImgGoal()
    #     state, result = self.getPeopleName(goalPeopleName, timeout)
    #     return state, result
    #
    # def getPeopleNameFromImgPath(self, img_path, timeout):
    #     img_loaded = cv2.imread(img_path)
    #     img_msg = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
    #     goalPeopleName = ProcessPeopleFromImgGoal(img=img_msg)
    #     state, result = self.getPeopleName(goalPeopleName, timeout)
    #     return state, result
    #
    # def getPeopleName(self, goalPeopleName, timeout):
    #     try:
    #         rospy.loginfo("### GET PEOPLE NAME ACTION PENDING")
    #         # send the current goal to the action server
    #         self._actionGetPeopleName_server.send_goal(goalPeopleName)
    #         # wait action server result
    #         finished_before_timeout = self._actionGetPeopleName_server.wait_for_result(rospy.Duration.from_sec(timeout))
    #         state = self._actionGetPeopleName_server.get_state()
    #         result = self._actionGetPeopleName_server.get_result()
    #         rospy.loginfo("###### GET PEOPLE NAME ACTION END , State: %s",str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionGetPeopleName_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state, result
    #     except Exception as e:
    #          rospy.logwarn("###### GET PEOPLE NAME FAILURE , State: %s",str(e))
    #     return GoalStatus.ABORTED, None
