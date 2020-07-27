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





class InspectionScenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None

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


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the InspectionScenario Scenario...")
        rospy.loginfo("######################################")
        

        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It0",60.0)
        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It1",60.0)
        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It2",60.0)
        rospy.sleep(10)
        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It3",60.0)
        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It4",60.0)
        self.sendNavOrderAction("NP","CRRCloseToGoal","INSPECTION_It5",60.0)

        #self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It0",60.0)
        #self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It1",60.0)
        #self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It2",60.0)
        #rospy.sleep(10)
        #self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It3",60.0)
        #self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It4",60.0)

        #for i in range(0,30):
        #    self.sendNavOrderAction("NP","CleanAndRetry","INSPECTION_It3",60.0)
        #    rospy.sleep(1)

        #self.sendNavOrderAction("NP","CRRCloseToGoal","A",60.0)
        #self.sendTtsOrderAction("TTS","Hello I Am an action text to speech","NO_WAIT_END","English",60.0)


    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):
        self._enableNavAction=True
        self._enableTtsAction=False
        self._enableDialogueAction=False
        self._enableAddInMemoryAction=False
        self._enableObjectDetectionMngAction=False
        AbstractScenarioAction.configure_intern(self)


