#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
from collections import namedtuple
import json
import actionlib
from robocup_msgs.msg import gm_bus_msg
from tts_hri.msg import TtsHriAction
from threading import Timer

######### Command to Test
## 
## rostopic pub /gm_bus_command robocup_msgs/gm_b_msg "{'action': 'TTS', 'action_id': '1', 'payload': '{\"txt\":\"I am alive, test for pepper TTS of HRI module\",\"lang\":\"English\", \"mode\":\"NO_WAIT_END\"}' , 'result': 0}"
#########


class TtsHri:
    TTS_ACTION="TTS"
    ANIMATED_TTS_ACTION="ANIMATED_TTS"
    NO_WAIT_END_MODE="NO_WAIT_END"
    WAIT_END_MODE="WAIT_END"
    WAIT_END_STATUS="WAIT_END_STATUS"
    NONE_STATUS="NONE_STATUS"
    _status=NONE_STATUS
    _currentOrder=None
    _session=None
    _memory=None
    _tts=None
    _maxWaitTimePerCall=60
    _timeout_checker=False
    _t_timer=''
    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

    def configureNaoqi(self):
        self._session = qi.Session()
        try:
            self._session.connect("tcp://" + ip + ":" + str(port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
         # Get the service ALTextToSpeech.
        self._tts = self._session.service("ALTextToSpeech")
        self._tts.setLanguage("English")
        # Say Emile in english
        self._tts.say("Ready to receive general manager order")

        self._animated_tts = self._session.service("ALAnimatedSpeech")

        # set the local configuration
        self.animated_tts_configuration = {"bodyLanguageMode":"contextual"}

        self._memory = self._session.service("ALMemory")
        #NOT WORK ???
        #self.subscriber = self._memory.subscriber("ALTextToSpeech/TextDone")
        #self.subscriber.signal.connect(self.onTextDone)

        self.subscriber2 = self._memory.subscriber("ALTextToSpeech/Status")
        self.subscriberAnimatedSpeechEnd= self._memory.subscriber("ALAnimatedSpeech/EndOfAnimatedSpeech")
        self.subscriber2.signal.connect(self.onTextStatus)
        self.subscriberAnimatedSpeechEnd.signal.connect(self.onEndAnimatedSpeech)


        return True

    def configure(self):
         # initialize services and topics as well as function calls
        self._gm_bus_pub = rospy.Publisher("gm_bus_answer", gm_bus_msg, queue_size=1)
        self._gm_bus_sub = rospy.Subscriber("gm_bus_command", gm_bus_msg, self.gmBusListener)

        # create action server and start it
        self._actionServer = actionlib.SimpleActionServer('tts_hri', TtsHriAction, self.executeTtsActionServer, False)
        self._actionServer.start()

        self._status=self.NONE_STATUS

    ###
    # expected msg msg.action='TTS' payload={'txt':'text to say','lang':'English', 'mode':'WAIT_END'}
    # expected msg msg.action='TTS' payload={'txt':'text to say','lang':'French', 'mode':'NO_WAIT_END'}
    ###
    def gmBusListener(self,msg):
        # check current order is TTS
        if msg.action== self.TTS_ACTION or msg.action== self.ANIMATED_TTS_ACTION:

            #check if current session is connected
            if not self._session.isConnected():
                rospy.loginfo("ERROR naoqi session not connected")
                self.configureNaoqi()

            #register current order
            self._currentOrder=msg
            
            
            #stop all previous TTS
            self._tts.stopAll()

            #load payload
            payloadObj=self.processPayload(msg.payload)

            if payloadObj==None:
                #FIXME return failure
                return
            isWaitForResult=False
            if self.WAIT_END_MODE == payloadObj.mode:
                isWaitForResult=True
                
            #start text to speech
            try :
               
                if isWaitForResult:
                    self._status=self.WAIT_END_STATUS
                if msg.action==self.ANIMATED_TTS_ACTION:
                     rospy.loginfo("ANIMATED_TTS: text to say:%s",str(payloadObj.txt))
                     self._animated_tts.say(payloadObj.txt,self.animated_tts_configuration)
                else:
                    rospy.loginfo("TTS: text to say:%s",str(payloadObj.txt))
                    self._tts.say(payloadObj.txt, payloadObj.lang)
            except RuntimeError:
                rospy.logwarn(str(payloadObj.lang)+" language is not installed, please install it to have a "+str(payloadObj.lang)+" pronunciation.")
                self._tts.say(payloadObj.txt, "English")

            if not isWaitForResult:
                self._currentOrder.result=3
                rospy.loginfo("TTS: text Done, No Wait")
                self._gm_bus_pub.publish(self._currentOrder)
                self._status=self.NONE_STATUS
                self._currentOrder=None


    #def onTextDone(self,data):
    #    print 'onTextDone:'+str(data)
    #    if self._status==self.WAIT_END_STATUS:
    #         self._currentOrder.result=3
    #         rospy.loginfo("TTS: text Done")
    #         self._gm_bus_pub.publish(self._currentOrder)
    #         self._status=self.NONE_STATUS
    #         self._currentOrder=None
             
    def onTextStatus(self,status):
        #print str(status)+'\n'
        if self._status==self.WAIT_END_STATUS:
             self._timeout_checker=True
             if status[1]=='done':
                if self._currentOrder != None :
                    self._currentOrder.result=3
                    rospy.loginfo("TTS: text Done")
                    self._gm_bus_pub.publish(self._currentOrder)
                    self._status=self.NONE_STATUS
                    self._currentOrder=None
   
    def onEndAnimatedSpeech(self,status):
        print str(status)
        if self._status==self.WAIT_END_STATUS:
             self._timeout_checker=True
             #if status[1]=='done':
             if self._currentOrder != None :
                self._currentOrder.result=3
                rospy.loginfo("Animated Speech End: text Done")
                self._gm_bus_pub.publish(self._currentOrder)
                self._status=self.NONE_STATUS
                self._currentOrder=None


    def processPayload(self,payload):
        try:
            #rospy.logwarn("payload: %s",str(payload))
            jsonObject = json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            #rospy.logwarn("jsonObject: %s",str(jsonObject))
            jsonObject.txt
            jsonObject.mode
            jsonObject.lang
            #rospy.logwarn("Object ready txt:%s",str(jsonObject.txt))
            return jsonObject
        except Exception as e:
            rospy.logwarn("Unable to load TTS payload: %s" % e)
            return None
  

    def executeTtsActionServer(self, goal):
        isActionSucceed=True
        try:
            # call the process associating to the action
            # caution msg is publish to the gm_answer... needed ??
            
            #stop all previous TTS
            self._tts.stopAll()

            isWaitForResult=False
            if self.WAIT_END_MODE == goal.mode:
                isWaitForResult=True

            #start text to speech
            try :

                
                if isWaitForResult:
                    self._status=self.WAIT_END_STATUS
                if goal.action == self.ANIMATED_TTS_ACTION:
                    rospy.loginfo("ANIMATED_TTS: text to say:%s",str(goal.txt))
                    self._tts.setLanguage(goal.lang)
                    self._animated_tts.say(goal.txt,self.animated_tts_configuration)
                else:
                    rospy.loginfo("TTS: text to say:%s",str(goal.txt))
                    self._tts.say(goal.txt, goal.lang)
            except RuntimeError:
                rospy.logwarn(str(goal.lang)+" language is not installed, please install it to have a "+str(goal.lang)+" pronunciation.")
                self._tts.say(goal.txt, "English")
            if not isWaitForResult:
                isActionSucceed=True
            else:
                # set a timer
                # check if event trigged if yes return success
                self.timeout_checker=False
                self._t_timer = Timer(self._maxWaitTimePerCall, self._timeout_checker)
                self._t_timer.start()
                while not self.timeout_checker:
                    rospy.sleep(0.1)
                isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded()
        else:
            self._actionServer.set_aborted()



if __name__ == "__main__":
    rospy.init_node('pepper_tts_hri')
    ip=rospy.get_param('~ip',"192.168.0.189")
    port=rospy.get_param('~port',9559)
   
    
    TtsHri(ip,port)
    rospy.spin()

