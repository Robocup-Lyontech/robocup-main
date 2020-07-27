#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
import yaml
from collections import namedtuple
import json
import actionlib
from sensor_msgs.msg import Joy
from tts_hri.msg import TtsHriAction
from threading import Timer


class TtsJoy:
    """ DEPRECATED: use TtsJoyImage instead """
    _lastButton=None
    _lastPressTime=0
    _currentTxtIndex=-1
    MIN_BUTTON_PRESSED_TIME=1
    
    def __init__(self,ip,port,config_folder,scenario_file):
        self._ip=ip
        self._port=port
        self._config_folder=config_folder
        self._scenario_file=scenario_file
        while not self.configureNaoqi() and not not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

    def configureNaoqi(self):
        self._session = qi.Session()

        # set the local configuration
        self.animated_tts_configuration = {"bodyLanguageMode":"contextual"}

        try:
            self._session.connect("tcp://" + ip + ":" + str(port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
         # Get the service ALTextToSpeech.
        self._tts = self._session.service("ALTextToSpeech")
        self._tts.setLanguage("English")
        self._tts.say("Ready to receive order for dialogue")

        self._animated_tts = self._session.service("ALAnimatedSpeech")

        self._memory = self._session.service("ALMemory")
        #NOT WORK ???
        #self.subscriber = self._memory.subscriber("ALTextToSpeech/TextDone")
        #self.subscriber.signal.connect(self.onTextDone)

        self.subscriber2 = self._memory.subscriber("ALTextToSpeech/Status")
        self.subscriberAnimatedSpeechEnd= self._memory.subscriber("ALAnimatedSpeech/EndOfAnimatedSpeech")

        #change collision distance
        motion_service  = self._session.service("ALMotion")

        motion_service.setOrthogonalSecurityDistance(0.05)
        motion_service.setTangentialSecurityDistance(0.05)

        self._autolife_service = self._session.service("ALAutonomousLife")
        self._posture_service = self._session.service("ALRobotPosture")
        

        return True

    def configure(self):
         # initialize services and topics as well as function calls
        self._joy_sub = rospy.Subscriber("/joy",Joy, self.joyListener,queue_size=1)
        self.processPayload(self._config_folder+"/"+self._scenario_file)
        self._lastPressTime = time.time()
        try:
            self._tts.setLanguage(self._lang)
        except Exception as e:
                rospy.logwarn(str(self._lang)+" language is not installed, please install it to have a "+str(self._lang)+" pronunciation. e:"+str(e))

    ###
    # expected msg msg.action='TTS' payload={'txt':'text to say','lang':'English', 'mode':'WAIT_END'}
    # expected msg msg.action='TTS' payload={'txt':'text to say','lang':'French', 'mode':'NO_WAIT_END'}
    ###
    def joyListener(self,msg):
        current_button_time = time.time()
        
        if msg.buttons[0] == 1:
            if self.needToProcess(0,current_button_time):
                if self._currentTxtIndex -1 >= 0:
                        self.sayAnimated(self._txt[self._currentTxtIndex -1],self.animated_tts_configuration)
                        self._currentTxtIndex =self._currentTxtIndex -1
                else:
                        self.sayAnimated(self._txt[0],self.animated_tts_configuration)
        elif msg.buttons[1] == 1:
            if self.needToProcess(1,current_button_time):
                if self._currentTxtIndex +1 < len(self._txt):
                        self.sayAnimated(self._txt[self._currentTxtIndex +1],self.animated_tts_configuration)
                        self._currentTxtIndex =self._currentTxtIndex +1
                else:
                        self.sayAnimated(self._txt[len(self._txt)-1],self.animated_tts_configuration)
        elif msg.buttons[2] == 1:
            if self.needToProcess(2,current_button_time):
                self._currentTxtIndex =-1
        elif msg.buttons[3] == 1:
            if self.needToProcess(3,current_button_time):
                self._currentTxtIndex =len(self._txt)
                self.sayAnimated("",{"bodyLanguageMode":"disabled"})
        elif msg.buttons[4] == 1:
            if self.needToProcess(3,current_button_time):
                self.toogleAnimatedMode()
        

    def needToProcess(self,button_index,current_button_time):
        need_to_process=False
        if self._lastButton == button_index :
            if current_button_time - self._lastPressTime  > self.MIN_BUTTON_PRESSED_TIME :
                    need_to_process=True
        else:
            need_to_process=True
            self._lastButton = button_index
            self._lastPressTime=current_button_time
        return need_to_process
    
    def toogleAnimatedMode(self):
        rospy.logwarn("BEFORE: STATE :"+str(self._autolife_service.getState()))
        if  self._autolife_service.getState() != 'disabled':
            self._autolife_service.setState('disabled')
            self._posture_service.goToPosture("Stand",0.3)
        else:
            self._autolife_service.setState('solitary')
        rospy.logwarn("AFTER: STATE :"+str(self._autolife_service.getState()))

    def sayAnimated(self, txt,animated_tts_configuration):
        try:
            rospy.loginfo("TXT:"+str(txt))
            self._animated_tts.say(txt,animated_tts_configuration)
        except Exception as e:
            rospy.logwarn("Unable use Animated Speech: %s" % e)

    def processPayload(self,payload):
        try:
            with open(payload) as json_data:
                jsonContent = yaml.safe_load(json_data)
                rospy.loginfo("Scenario configuraiton file content: %s" % jsonContent)
                #jsonString=json.load(jsonContent)
                #jsonObject = json.load(jsonString, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
                ##rospy.loginfo("Scenario configuraiton Object content: %s" % str(jsonObject))
                self._action=jsonContent['action']
                rospy.loginfo('action: '+jsonContent['action'])
                self._description=jsonContent['description']
                rospy.loginfo('description: '+jsonContent['description'])
                self._txt=[]
                for s in jsonContent['txt']:
                    self._txt.append(s.encode('utf8'))
                
                #self._txt=jsonContent['txt']
                rospy.loginfo('txt: '+str(jsonContent['txt']))
                self._lang=jsonContent['lang']
        except Exception as e:
            rospy.logwarn("Unable to load TTS payload: %s" % e)
            return None

if __name__ == "__main__":
    rospy.init_node('pepper_tts_hri')
    ip=rospy.get_param('~ip',"192.168.0.189")
    port=rospy.get_param('~port',9559)
    config_folder=rospy.get_param('~config_folder',"/home/astro/catkin_robocup2018/data/tts_dialogue")
    scenario_file=rospy.get_param('~scenario_file',"room_presentation.json")
   
    
    TtsJoy(ip,port,config_folder,scenario_file)
    rospy.spin()