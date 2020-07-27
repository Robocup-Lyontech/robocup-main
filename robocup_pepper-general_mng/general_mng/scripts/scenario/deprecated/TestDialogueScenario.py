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





class TestDialogueScenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

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
        rospy.loginfo("Starting the TestDialogueScenario Scenario...")
        rospy.loginfo("######################################")
        
        #TOO make the logic of the scenario

        orderState1=self.sendDialogueOrderAction("Cocktail/ScenarioStart","",60.0)
        rospy.loginfo("-------> OK, wait 20s")
        rospy.sleep(20.0)
        orderState2=self.sendDialogueOrderAction("Cocktail/OrdersStart","Cocktail/OrdersFinish",60.0*3)
        rospy.loginfo("-------> OK, wait 20s")
        
        rospy.sleep(20.0)
        orderState2=self.sendDialogueOrderAction("Cocktail/OrdersCheckStart","Cocktail/OrdersCheckFinish",60.0)
        

        #/Cocktail/SearchAndInformStart




    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):
        self._enableDialogueAction=True
        self._enableNavAction=False
        self._enableTtsAction=False
        AbstractScenarioAction.configure_intern(self)



