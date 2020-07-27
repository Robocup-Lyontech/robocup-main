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





class TestCocktailPartyV1Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None
    HEAD_PITCH_FOR_SPEECH_POSE=-0.3
    HEAD_PITCH_FOR_NAV_POSE= 0.5
    HEAD_YAW_CENTER=0.0

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
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,HEAD_YAW_CENTER,True)
        rospy.sleep(2.0)
        orderState1,result1=self.sendDialogueOrderAction("Cocktail/ScenarioStart","",60.0)
        rospy.sleep(5.0)
             
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE,HEAD_YAW_CENTER,True)

        orderState2=self.sendNavOrderAction("NP","CRRCloseToGoal","G_R",60.0)
        #rospy.loginfo("-------> OK, wait 20s")
        #rospy.sleep(20.0)

        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,HEAD_YAW_CENTER,True)

        orderState3,result3=self.sendDialogueOrderAction("Cocktail/OrdersStart","Cocktail/OrdersFinish",60.0*3)

        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE,HEAD_YAW_CENTER,True)
        
        
        orderState4=self.sendNavOrderAction("NP","CRRCloseToGoal","E_R",60.0)
        
        #rospy.loginfo("-------> OK, wait 20s")
        #rospy.sleep(20.0)
        orderState5,result5=self.sendDialogueOrderAction("Cocktail/OrdersCheckStart","Cocktail/OrdersCheckFinish",60.0)
        

        #/Cocktail/SearchAndInformStart




    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):
        self._enableDialogueAction=True
        self._enableNavAction=True
        self._enableTtsAction=False
        AbstractScenarioAction.configure_intern(self)

    def moveheadPose(self,pitch_value,yaw_value,track):
        try:
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            result=self._moveHeadPose(pitch_value,yaw_value,track)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return
        
