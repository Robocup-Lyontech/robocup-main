#!/usr/bin/env python  

import qi
import argparse
import sys
import time
from threading import Timer
import rospy
from collections import namedtuple
import json
import actionlib
from actionlib_msgs.msg import GoalID
from robocup_msgs.msg import gm_bus_msg
from dialogue_hri_actions.msg import DialogueSendSignalAction,DialogueSendSignalResult,AddInMemoryAction, AddInMemoryResult

######### Command to Test
## 
## 
#########


class DialogueHri:
    _maxWaitTimePerCall=40.0
    NO_WAIT_END_MODE="NO_WAIT_END"
    WAIT_END_MODE="WAIT_END"
    WAIT_SIGNAL_STATUS="WAIT_SIGNAL"
    RECEIVED_SIGNAL_STATUS="RECEIVED_SIGNA"
    NONE_STATUS="NONE_STATUS"
    _status=NONE_STATUS
    _currentSignalSubscriber=None
    _currentSignalSubName=None
    _is_current_result_succeed=False
    _current_result_value=None
    _session=None
    _memory=None

    _t_timer=''
    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

        rospy.loginfo("DIALOGUE: READY TO PROCESS ACTION")

    def configureNaoqi(self):
        self._session = qi.Session()
        try:
            self._session.connect("tcp://" + ip + ":" + str(port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
        self._memory = self._session.service("ALMemory")
        rospy.loginfo("DIALOGUE: CONFIGURATION NAOQI OK")
        return True

    def configure(self):
        # create action server and start it
        
        #self._actionStartServer = actionlib.SimpleActionServer('dialogue_hri_start', DialogueStartScenarioAction, self.executeDialogueStartActionServer, False)
        #self._actionStartServer.start()

        self._actionServerDialogue = actionlib.SimpleActionServer('dialogue_hri_signal', DialogueSendSignalAction, self.executeDialogueSignalActionServer, False)
        self._actionServerDialogue.start()

        self._actionServerAddMemory = actionlib.SimpleActionServer('add_in_memory_action', AddInMemoryAction, self.executeAddInMemoryServer, False)
        self._actionServerAddMemory.start()

        self._status=self.NONE_STATUS

        self._action_cancel_sub = rospy.Subscriber("/dialogue_hri_signal/cancel", GoalID, self.cancelOrder)

        rospy.loginfo("DIALOGUE: CONFIGURATION ACTION OK")

  
             
    def onSignalCallback(self,status):
        rospy.loginfo(" SIGNAL CALLBACK: "+str(status))
        #FIXME check that feedback are tab
        if status==0:
            self._is_current_result_succeed=False
            self._current_result_value=None
        elif status==1:
            self._is_current_result_succeed=True
            self._current_result_value=None
        else:
            self._is_current_result_succeed=True
            self._current_result_value=status
             
        self._status=self.RECEIVED_SIGNAL_STATUS
        
    #def processPayload(self,payload):
    #    try:
    #        #rospy.logwarn("payload: %s",str(payload))
    #        jsonObject = json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
    #        #rospy.logwarn("jsonObject: %s",str(jsonObject))
    #        jsonObject.txt
    #        jsonObject.mode
    #        jsonObject.lang
    #        #rospy.logwarn("Object ready txt:%s",str(jsonObject.txt))
    #        return jsonObject
    #    except Exception as e:
    #        rospy.logwarn("Unable to load TTS payload: %s" % e)
    #        return None
  



    #def executeDialogueStartActionServer(self, goal):
    #    isActionSucceed=False
    #    try:
    #        try :
    #            #TODO SEND START SIGNAL
    #            
    #            isActionSucceed=True
    #        except RuntimeError as e:
    #            rospy.logwarn("Error occurs when sending signal:"+str(e))
    #    except Exception as e:
    #        rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
    #    if isActionSucceed:
    #        self._actionServer.set_succeeded()
    #    else:
    #        self._actionServer.set_aborted()
 

    def executeDialogueSignalActionServer(self, goal):
        rospy.loginfo("DIALOGUE: Action received signal_to_emit:"+str(goal.signal_to_emit)+", signal_to_wait:"+str(goal.signal_to_wait))
        isActionSucceed=False
        waitMode=self.NO_WAIT_END_MODE
        try:

            #check if signal_to_wait is set
            if goal.signal_to_wait != '':
                #define new subscriber to signal from naoqi
                self._currentSignalSubscriber = self._memory.subscriber(goal.signal_to_wait)
                self._currentSignalSubscriber.signal.connect(self.onSignalCallback)
                self._currentSignalSubName=goal.signal_to_wait
                #need to wait signal before sending action end
                waitMode=self.WAIT_END_MODE

            #start to emit signal
            try :
                rospy.loginfo("DIALOGUE: emit on signal :%s",str(goal.signal_to_emit))
                if self.WAIT_END_MODE == waitMode:
                    self._status=self.WAIT_SIGNAL_STATUS

                ##EMIt SIGNAL TO NAOQI
                self._memory.raiseEvent(goal.signal_to_emit,1)

                if self.NO_WAIT_END_MODE == waitMode:
                     isActionSucceed=True
                     self._is_current_result_succeed=True
                else:
                     # set a timer
                     # check if event trigged if yes return success
                     #self.timeout_checker=False
                     #self._t_timer = Timer(self._maxWaitTimePerCall, self._timeout_checker_fct)
                     #self._t_timer.start()
                     #while not self.timeout_checker:
                     #while self._status is self.WAIT_SIGNAL_STATUS and not self.timeout_checker and  not rospy.is_shutdown():
                     while self._status is self.WAIT_SIGNAL_STATUS and  not rospy.is_shutdown():

                        #check if another call is asked until the current one is not complet
                        if self._actionServerDialogue.is_preempt_requested():
                            rospy.loginfo('%s: Preempted-----------------------------------')
                            self.cancelOrder(None)

                        rospy.sleep(0.1)

                     #if self.timeout_checker:
                     #    #END of Internal Timeout
                     #    isActionSucceed=False
                     #else:
                     #    #END of Internal Timeout
                     #    isActionSucceed=True
                isActionSucceed=True
            except RuntimeError as e:
                rospy.logwarn("ERROR during sending signal:"+str(goal.signal_to_emit)+",e:"+str(e))
            
            #
            
        except Exception as e:
            rospy.logwarn("Error during SIgnal configuration process:, error:[%s]", str(e))
            self._currentSignalSubName=None
            self._currentSignalSubscriber=None
        
        result=DialogueSendSignalResult()
        if self._is_current_result_succeed:
            result.result=3
            if self._current_result_value!=None:
                result.payload=self._current_result_value
        else:
            result.result=4
            isActionSucceed=False

        if isActionSucceed:

            self._actionServerDialogue.set_succeeded(result)
        else:
            self._actionServerDialogue.set_aborted(result)

        


    def executeAddInMemoryServer(self, goal):
        rospy.loginfo("DIALOGUE: Action received Add to memmory action,location :"+str(goal.memory_location)+", payload:"+str(goal.payload))
        isActionSucceed=False
        try:
            #FIXME TO DO
            self._memory.insertData(goal.memory_location,goal.payload)
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("ERROR during setting payload in memory:"+str(goal.memory_location)+",e:"+str(e))
            self._currentSignalSubName=None
            self._currentSignalSubscriber=None

        result=AddInMemoryResult()
        if isActionSucceed:
            result.result=3
            self._actionServerAddMemory.set_succeeded(result)
        else:
            result.result=4
            self._actionServerAddMemory.set_aborted(result)
    
    def _timeout_checker_fct(self):
        self.timeout_checker=True

    def cancelOrder(self,goalId):
        rospy.logwarn("DIALOGUE: Cancel order asked on current action")
        self._status=self.NONE_STATUS
        self._is_current_result_succeed=False



if __name__ == "__main__":
    rospy.init_node('pepper_dialogue_hri')
    ip=rospy.get_param('~ip',"192.168.1.201")
    port=rospy.get_param('~port',9559)
   
    DialogueHri(ip,port)
    rospy.spin()

