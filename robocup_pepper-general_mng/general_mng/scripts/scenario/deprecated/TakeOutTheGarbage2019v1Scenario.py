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
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt,GoToCarryPose,ReleaseArms

class TakeOutTheGarbage2019Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None

    def __init__(self,config):
        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)

        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the TakeOutTheGarbage2019 Scenario...")
        rospy.loginfo("######################################")
        
        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","D",120.0)
        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","C",120.0)
        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","B",120.0)
        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","I",120.0)
        orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","It0",120.0)
        orderState1=self.sendNavOrderAction("NP","CRRCloseToGoal","It1",120.0)

        self.sendTtsOrderAction("TTS","Ok , I see the garbage, can you please give me the garbage as explain on the tablet ? " ,"NO_WAIT_END","English",60.0*2)
        
        #Be ready to take garbage
        self.poseToTakeGarbage()
        
        #CALL HRI to Garbage interaction
        #Wait HRI confirmation from Almemory
        orderState2,result2=self.sendDialogueOrderAction("Garbage/PlaceGarbageStart","Garbage/PlaceGarbageFinished",10.0*1)
        rospy.loginfo(result2)

        #Go to carry Pose
        self.poseToCarryGarbage()

        self.sendTtsOrderAction("TTS"," Every thing is ready ? " ,"NO_WAIT_END","English",60.0)

        orderState3,result3=self.sendDialogueOrderAction("Garbage/CheckBeforeGoStart","Garbage/CheckBeforeGoFinished",10.0*1)
        rospy.loginfo(result3)


        self.sendTtsOrderAction("TTS"," Let's go! " ,"NO_WAIT_END","English",60.0)

        #Navigate to destination
        orderState4=self.sendNavOrderAction("NP","CRRCloseToGoal","It0",120.0)

        #Release Garbage
        self.poseToTakeGarbage()


        #Release Arm at the end
        self.releaseArms()
        
        



    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)

    def initScenario(self):
        AbstractScenarioAction.configure_intern(self)
        self.initPoseToCarryGarbage()
        self.initPoseToTakeGarbage()
        self.initReleaseArms()


    def initPoseToCarryGarbage(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/go_to_carry_pose', 5)
            rospy.loginfo("     service /go_to_carry_pose wait time ready")
            self._gotoCarryPose = rospy.ServiceProxy('/go_to_carry_pose', GoToCarryPose)
        except Exception as e:
            rospy.logerr("      Service /go_to_carry_pose call failed: %s" % e)
            return


    def initPoseToTakeGarbage(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/move_arm_hand', 5)
            rospy.loginfo("     service /move_arm_hand wait time ready")
            self._gotoTakePose = rospy.ServiceProxy('/move_arm_hand', MoveArmHand)
        except Exception as e:
            rospy.logerr("      Service /move_arm_hand call failed: %s" % e)
            return

    def initReleaseArms(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/release_arms', 5)
            rospy.loginfo("     service /release_arms wait time ready")
            self._releaseArms = rospy.ServiceProxy('/release_arms', ReleaseArms)
        except Exception as e:
            rospy.logerr("      Service /release_arms call failed: %s" % e)
            return
    

    def poseToTakeGarbage(self):
        result = self._gotoTakePose(arm_l_or_r='l',turn_rad=0.0,stiffness=0.7)

    def poseToCarryGarbage(self):
        result = self._gotoCarryPose(arm_l_or_r='l',keep_pose=True,stiffness=0.9)

    def releaseArms(self):
        result = self._releaseArms(stiffness=0.7)


         


