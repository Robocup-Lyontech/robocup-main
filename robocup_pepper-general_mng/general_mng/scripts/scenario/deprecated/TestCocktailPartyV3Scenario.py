__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import uuid
import time
import random
import actionlib

from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenario import AbstractScenario


from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from pepper_pose_for_nav.srv import MoveHeadAtPosition





class TestCocktailPartyV3Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None
    HEAD_PITCH_FOR_SPEECH_POSE=-0.3
    HEAD_PITCH_FOR_NAV_POSE= 0.5
    HEAD_YAW_CENTER=0.0
    DEFAULT_OBJ_LABEL=[]
    DEFAULT_OBJECT_MEMORY_LOCATION="Cocktail/Objects"

    def __init__(self,config):
        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)
        #try:
        #    self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
        #    self._actionNavMng_server.wait_for_server()
        #    self._actionTtsHri_server = actionlib.SimpleActionClient('tts_hri', TtsHriAction)
        #    self._actionTtsHri_server.wait_for_server()
        #except Exception as e:
        #    rospy.loginfo("Unable to connect to action server: %s" % e)

        #FIXME wait the service ?
        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)

        try:
            self.obj_labels=config['labels']
        except Exception as e:
            rospy.logwarn("no config value for obj_label use default:"+str(self.DEFAULT_OBJ_LABEL))
            self.obj_labels=self.DEFAULT_OBJ_LABEL

        try:
            self.object_memory_location=config['object_memory_location']
        except Exception as e:
            rospy.logwarn("no config value for object_memory_location use default:"+str(self.DEFAULT_OBJECT_MEMORY_LOCATION))
            self.object_memory_location=self.DEFAULT_OBJECT_MEMORY_LOCATION




        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the TEST_COCKTAIL_PARTY_V1 Scenario...")
        rospy.loginfo("######################################")

        #TOO make the logic of the scenario

        ### 0-SET HEAD FOR NAVIGATION
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,self.HEAD_YAW_CENTER,True)
        rospy.sleep(2.0)

        ### 1-INFORM NAOQI TO START DIALOGUE
        orderState1,result1=self.sendDialogueOrderAction("Cocktail/ScenarioStart","",60.0)
        rospy.sleep(5.0)

        ### 2-SET HEAD FOR NAVIGATION
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE,self.HEAD_YAW_CENTER,True)

        ### 3-NAVIGATE TO COMMAND AREA
        orderState2=self.sendNavOrderAction("NP","CRRCloseToGoal","A",60.0)
        #rospy.loginfo("-------> OK, wait 20s")
        #rospy.sleep(20.0)

        ### 2-SET HEAD FOR DIALOGUE
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,self.HEAD_YAW_CENTER,True)

        ### 4-INFORM NAOQI TO START TO GET COMMAND
        orderState3,result3=self.sendDialogueOrderAction("Cocktail/OrdersStart","Cocktail/OrdersFinish",60.0*3)

        # LOOP 3 Times
        #CALL PEOPLE DESCRIPTION

        #GET PEOPLE WITH GREATEST BOUNDING BOX

        #ADD IN NAOQI MEMORY

        #INFORM NAOQI DETECTION READY


        ### 5-SET HEAD FOR NAVIGATION
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE,self.HEAD_YAW_CENTER,True)

        ### 6-NAVIGATE TO BAR TENDER
        orderState4=self.sendNavOrderAction("NP","CRRCloseToGoal","G",60.0)

        ### 7-INFORM NAOQI TO TELL COMMAND TO BAR TENDER
        orderState5,result5=self.sendDialogueOrderAction("Cocktail/OrdersCheckStart","Cocktail/OrdersCheckFinish",60.0*3)


         ### 6-NAVIGATE TO BOTTLE LOCATION
        orderState6=self.sendNavOrderAction("NP","CRRCloseToGoal","H",60.0)

        ### 8-DETECTION OBJECTS
        orderState7,result7=self.getObjectInFrontRobot(self.obj_labels, True, 60.0)
        rospy.loginfo("#### OBJECT DETECTED ####")
        rospy.loginfo(result0)

        ### 9-ADD OBJECTS INTO NOAQI MEMORY
        orderState8,result8=self.addInPepperMemory(self.object_memory_location,str(result0.labelList),5.0)


        ### 11-NAVIGATE TO COMMAND AREA
        orderState9=self.sendNavOrderAction("NP","CRRCloseToGoal","A",60.0)

        ### 11BIS-INFORM NAOQI THAT ROBOT LOOKING FOR PERSON
        #orderState3,result3=self.sendDialogueOrderAction("Cocktail/SearchAndInformStart","Cocktail/StopAndInformFinish",60.0*3)


        ### 12-INFORM NAOQI THAT PERSON IS FOUND
        orderState10,result10=self.sendDialogueOrderAction("Cocktail/FindAndInformStart","Cocktail/FindAndInformFinish",60.0*3)

        ### 13-INFORM NAOQI TO TELL COMMAND TO BAR TENDER
        orderState11,result11=self.sendDialogueOrderAction("Cocktail/OrdersCorrectionStart","Cocktail/OrdersCorrectionFinish",60.0*3)





    def gmBusListener(self,msg):
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):

        self._enableNavAction=True
        self._enableTtsAction=False
        self._enableDialogueAction=True
        self._enableAddInMemoryAction=True
        self._enableObjectDetectionMngAction=True

        AbstractScenarioAction.configure_intern(self)

    def moveheadPose(self,pitch_value,yaw_value,track):
        try:
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            result=self._moveHeadPose(pitch_value,yaw_value,track)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return
