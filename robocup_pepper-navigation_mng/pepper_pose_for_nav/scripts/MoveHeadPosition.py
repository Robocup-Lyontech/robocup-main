#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from geometry_msgs.msg import Twist

class HeadFix():
    _continuous_fix_activated=True
    HEAD_PITCH_OBSTACLE=0.6
    HEAD_YAW_OBSTACLE_LEFT=0.7
    HEAD_YAW_OBSTACLE_RIGHT=-0.7

    def __init__(self,session):
        """
        This example uses the setExternalCollisionProtectionEnabled method.
        """
        # Get the service ALMotion.
        self._motion_service  = session.service("ALMotion")
        self._memory_service = session.service("ALMemory")

        self._autolife_service = session.service("ALAutonomousLife")
        self._basic_awareness_service = session.service("ALBasicAwareness")

        if self._autolife_service.getState() != 'disabled':
            self._autolife_service.setState('disabled')

        if self._autolife_service.getAutonomousAbilityEnabled("BasicAwareness") != 'disabled':
            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        
        self._posture_service = session.service("ALRobotPosture")
        #if self._posture_service.getPostureFamily() != "Stand" and self._posture_service.getPostureFamily() != "Standing":
        self._posture_service.goToPosture("Stand",0.3)

        self._fractionMaxSpeed = 0.2
        self._motion_service.setStiffnesses("HEAD", 0.8)
        self._motion_service.setStiffnesses("TORSO", 0.8)
        self._error=0.1
        #pitch_value=0.3
        self._pitch_value=0.0
        self._yaw_value=0.0
        self._isEnd=False
        #declare ros service 
        self.setHeadPositionSrv = rospy.Service('move_head_pose_srv', MoveHeadAtPosition, self.setHeadPositionSrvCallback)
        self.cmdSub=rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallBack)


    def fixHead(self):
        while not rospy.is_shutdown():
            if self._continuous_fix_activated:
                headYawPos = self._memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
                headPitchPos = self._memory_service.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
                #rospy.loginfo("headYawPos:"+str(headYawPos)+",headPitchPos:"+str(headPitchPos))
                if self.needToFixHead(headPitchPos,headYawPos):
                    self._motion_service.setAngles("HeadYaw", self._yaw_value, self._fractionMaxSpeed) 
                    self._motion_service.setAngles("HeadPitch", self._pitch_value, self._fractionMaxSpeed) ## fix head on the horizon 0.0, fix head looking for obstacle 0.3
                    rospy.logdebug("update head, headYawPos:"+str(headYawPos)+",headPitchPos:"+str(headPitchPos))
                
            time.sleep(0.1)

    def needToFixHead(self, headPitchPos,headYawPos):
        result=False
        if headPitchPos>(self._pitch_value+self._error) or headPitchPos<(self._pitch_value-self._error):
            result=True
        if headYawPos>(self._yaw_value+self._error) or headYawPos<(self._yaw_value-self._error):
            result=True
        return result

    def setHeadPositionSrvCallback(self,req):
        self._pitch_value=req.pitch_value
        self._yaw_value=req.yaw_value
        self._continuous_fix_activated=req.continuous_fix

        if not self._continuous_fix_activated:
            self._motion_service.setAngles("HeadYaw", self._yaw_value, self._fractionMaxSpeed)
            self._motion_service.setAngles("HeadPitch", self._pitch_value, self._fractionMaxSpeed)

            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", True)
            self._basic_awareness_service.setStimulusDetectionEnabled("Sound", False)
        else:
            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        return True

    def cmdVelCallBack(self, data):

        self._pitch_value=self.HEAD_PITCH_OBSTACLE

        # if the cmd turn on the right
        if data.angular.z <0:
            self._yaw_value=self.HEAD_YAW_OBSTACLE_RIGHT
        
        # if the cmd turn on the left
        elif data.angular.z >0:
            self._yaw_value=self.HEAD_YAW_OBSTACLE_LEFT

        elif data.angular.z == 0:
            self._yaw_value= 0.0


if __name__ == "__main__":
    rospy.init_node('pepper_head_pose_fix')
    ip=rospy.get_param('~ip',"192.168.1.189")
    port=rospy.get_param('~port',9559)
   
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    headFix=HeadFix(session)
    headFix.fixHead()
    # spin
    rospy.spin()



