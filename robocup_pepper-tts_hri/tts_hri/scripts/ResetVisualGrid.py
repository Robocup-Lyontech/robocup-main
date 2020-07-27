#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
from collections import namedtuple
import json
import actionlib
from naoqi import ALProxy


######### Command to Test
## 
## rostopic pub /gm_bus_command robocup_msgs/gm_b_msg "{'action': 'TTS', 'action_id': '1', 'payload': '{\"txt\":\"I am alive, test for pepper TTS of HRI module\",\"lang\":\"English\", \"mode\":\"NO_WAIT_END\"}' , 'result': 0}"
#########


class ResetVisualGrid:
   
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
        

    def configure(self):
         # initialize services and topics as well as function 
        #ALVisualSpaceHistoryProxy::resetGrid()

        try:
            # create proxy on ALMemory
            memProxy = ALProxy("ALVisualSpaceHistory",self._ip,self._port)
            memProxy.resetGrid()
            print 'done'

  
        except RuntimeError,e:
            # catch exception
            print "error insert data", e

  


if __name__ == "__main__":
    rospy.init_node('pepper_tts_hri')
    ip=rospy.get_param('~ip',"192.168.0.201")
    port=rospy.get_param('~port',9559)
   
    
    ResetVisualGrid(ip,port)
    rospy.spin()

