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
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt,GoToCarryPose,ReleaseArms
from std_msgs.msg import Empty
import threading


######### Command to Test
## rosservice call /point_at "{x: 10.0, y: 10.0, z: 0.0, move_head: true, move_arm: true, pose_duration: 8.0}"
## rosservice call /move_arm_hand "{arm_l_or_r: 'l',turn_rad: 0.0,stiffness: 0.8}"
## rosservice call /go_to_carry_pose "{arm_l_or_r:'l', keep_pose: true, stiffness: 0.8}"
#########


class MoveTurnRobot:
    _session=None
    _memory=None

    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

        rospy.loginfo("MoveHri: READY TO PROCESS ACTION")


    def configureNaoqi(self):
        self._session = qi.Session()
        rospy.loginfo("MoveHRI: try connecting to robot...")
        try:
            self._session.connect("tcp://" + self._ip + ":" + str(self._port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + self._ip + "\" on port " + str(self._port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
        self._motion = self._session.service("ALMotion")
        self._tracker_service = self._session.service("ALTracker")

        rospy.loginfo("MOVE HRI: CONFIGURATION NAOQI OK")
        return True

    def configure(self):
        #create Service
        self._activateMoveSound_service = rospy.Service('move_turn_service', MoveTurn, self.moveturn)
        self._move_arm_service = rospy.Service('move_arm_hand', MoveArmHand, self.moveArmHand)
        self._point_at_service = rospy.Service('point_at', PointAt, self.pointAt)
        self._go_to_carry_pose = rospy.Service('go_to_carry_pose', GoToCarryPose, self.gotoCarryPose)
        self._release_arms = rospy.Service('release_arms', ReleaseArms, self.releaseArmSrv)

        rospy.loginfo("MoveHri: CONFIGURATION ACTION OK")

    def moveturn(self,req):
        self._motion.moveTo(0,0,req.turn_rad)
        rospy.loginfo("MoveHri: ROTATION ASKED rad:"+str(req.turn_rad))
        return []

    def moveArmHand(self,req):
        self.release_fix_pose_thread()
        # go to an init head pose.
        if req.arm_l_or_r == "r":
            names = ["RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw"]
            name_hand="RHand"
            name_stiffness = ["RArm"]
            #angles = [req.turn_rad, -0.09, 0.26, -0.14, 1.74]-0.349066
            angles = [req.turn_rad, -0.349066, 0.26, -0.14, 1.74]
        else:
            names = ["LShoulderPitch","LShoulderRoll","LElbowRoll","LElbowYaw","LWristYaw"]
            name_hand = "LHand"
            name_stiffness = ["LArm"]
            #angles = [req.turn_rad, -0.09, 0.26, 0.035, -1.17]
            angles = [req.turn_rad, 0.349066, 0.26, 0.035, -1.17]

        self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)

        times = [2.0,1.0,2.0,2.0,2.0,2.0]
        isAbsolute = True
        self._motion.angleInterpolation(names, angles, times, isAbsolute)
        self._motion.angleInterpolation(name_hand, 0.83, 1.0, True)

        self._motion.stiffnessInterpolation(name_stiffness, req.stiffness, 0.2)


        rospy.loginfo("MoveHri: Move Arm ASKED rad:" + str(req.turn_rad)+", r or l :"+str(req.arm_l_or_r)+ "stiffness: "+str(req.stiffness))
        return []

    def pointAt(self, req):
        if req.move_head:
            #Parameters lookAt:
            # lookAt(const std::vector<float>& Position, const float& FractionMaxSpeed, const bool UseWholeBody)
            #   - Position - position 3D [x, y, z] in FRAME_TORSO.
            #   - FractionMaxSpeed - a fraction.
            #   - UseWholeBody - if true, use whole body constraints.
            self._tracker_service.lookAt([req.x, req.y, req.z],0.7,False)
        if req.move_arm:
            self.release_fix_pose_thread()
            if req.y > 0:
                name_stiffness = ["LArm"]
                name = name_stiffness[0]
                isRightArm=False
            else:
                name_stiffness = ["RArm"]
                name = name_stiffness[0]
                isRightArm = True

            self._motion.stiffnessInterpolation(name_stiffness, 0.8, 1.0)


            #Parameters pointAt:
            # pointAt(const std::string& Effector, const std::vector<float>& Position, const int& Frame, const float& FractionMaxSpeed)
            #   - Effector - effector name. Could be "Arms", "LArm", "RArm".
            #   - Position - position 3D [x, y, z].
            #   - Frame - position frame {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            #   - FractionMaxSpeed - a fraction.
            self._tracker_service.pointAt(name, [req.x, req.y, req.z],0, 0.7)

            #Release if pose_duration is positive or equal to zero
            if req.pose_duration > 0.0:
                rospy.sleep(req.pose_duration)
                self.releaseArm(isRightArm,0.6)
            else:
                rospy.logwarn("Pepper arm still pointing: call release_arms service with 0.6 stiffness to release")
        return []

    def releaseArm(self,isRightArm,stiffness):
        self.release_fix_pose_thread()
        if isRightArm:
            name_angle = ["RShoulderPitch", "RShoulderRoll"]
            name_stiffness = ["RArm"]
        else:
            name_angle = ["LShoulderPitch", "LShoulderRoll"]
            name_stiffness = ["LArm"]
        # move arm at rest position
        angles = [1.6, 0]
        times = [2.0, 3.0]
        isAbsolute = True
        self._motion.angleInterpolation(name_angle, angles, times, isAbsolute)
        # set stiffness to 0.0
        self._motion.stiffnessInterpolation(name_stiffness, stiffness, 1.0)


    def releaseArmSrv(self,req):
        self.releaseArm(True,req.stiffness)
        self.releaseArm(False,req.stiffness)
        return []



    def gotoCarryPose(self, req):
        #FIXME provide a seconde carry pose close to the robot legs
        leftArmEnable = True
        rightArmEnable = True
        if req.arm_l_or_r == "r":
            name_stiffness = ["RArm"]
            self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)
            self._motion.angleInterpolation("RHand", 0.0, 1.0, True)
            self._motion.angleInterpolation("RWristYaw", 1.73, 1.0, True)
            self._motion.angleInterpolation("RElbowYaw",  2.08, 1.0, True)
            self._motion.angleInterpolation("RElbowRoll", 1.48, 1.0, True)
            self._motion.angleInterpolation("RShoulderPitch",-0.47, 2.0, True)
            self._motion.angleInterpolation("RShoulderRoll", -0.09, 2.0, True)

            #self._motion.angleInterpolation("RWristYaw",-1.4137167, 1.0, True)
            self._motion.angleInterpolation(["RShoulderPitch","RWristYaw"], [-1.52,-1.4137167], [2.0,1.0], True)
            #self._motion.angleInterpolation("RShoulderPitch", -1.52, 2.0, True)
            self._motion.angleInterpolation("RShoulderRoll", 0, 2.0, True)
            self._motion.angleInterpolation("RElbowYaw",  0.96, 1.0, True)


            joint_names = ["RHand", "RWristYaw","RElbowYaw" "RElbowRoll", "RShoulderPitch", "RShoulderRoll"]
            joint_angles = [0.0, 1.73, 2.08, 1.48, -0.47, -0.09]
            rightArmEnable = False

        else:
            name_stiffness = ["LArm"]
            #FIXME to be tested and modify !!!!
            self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)
            self._motion.angleInterpolation("LHand", 0.0, 1.0, True)
            self._motion.angleInterpolation("LWristYaw", -1.73, 1.0, True)
            self._motion.angleInterpolation("LElbowYaw", -2.08, 1.0, True)
            self._motion.angleInterpolation("LElbowRoll", -1.48, 1.0, True)
            self._motion.angleInterpolation("LShoulderPitch", -0.47, 2.0, True)
            self._motion.angleInterpolation("LShoulderRoll", -0.09, 2.0, True)

            #self._motion.angleInterpolation("LWristYaw",  1.4137167, 1.0, True)
            self._motion.angleInterpolation(["LShoulderPitch","LWristYaw"], [-1.52,1.4137167], [2.0,1.0], True)
            self._motion.angleInterpolation("LShoulderRoll", 0, 2.0, True)
            self._motion.angleInterpolation("LElbowYaw", - 0.96 , 1.0, True)


            joint_names=["LHand","LWristYaw","LElbowYaw","LElbowRoll","LShoulderPitch","LShoulderRoll"]
            joint_angles=[0.0,-1.73,-2.08,-1.48,-0.47,-0.09]
            leftArmEnable = False


        if req.keep_pose:
            self._motion.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            #self.release_fix_pose_thread()
            #NO MORE NEED TO KEEP THE POSE --> disable arm pose for nav. instead
            # self.isEnd = False
            # self.keep_arm_pose_thread = threading.Thread(target=self.keepArmPose, args=(joint_names,joint_angles,name_stiffness,5,))
            # self.keep_arm_pose_thread.start()

        self._motion.stiffnessInterpolation(name_stiffness, req.stiffness, 0.2)
        return []


    def keepArmPose(self,joint_names,joint_angles,arm_name,check_frequency):
        rate = rospy.Rate(check_frequency)  # 10hz
        while not self.isEnd and not rospy.is_shutdown():
            # reset the stiffness of the arm
            self._motion.stiffnessInterpolation(arm_name, 0.8, 0.5)
            #0.2=fraction of the max speed
            self._motion.setAngles(joint_names, joint_angles, 0.2)
            rate.sleep()

    def release_fix_pose_thread(self):
        self._motion.setMoveArmsEnabled(True, True)
        if hasattr(self, 'keep_arm_pose_thread'):
            self.isEnd = True
            self.keep_arm_pose_thread.join(100)

if __name__ == "__main__":
    rospy.init_node('pepper_move_hri')
    ip=rospy.get_param('~ip',"192.168.42.221")
    port=rospy.get_param('~port',9559)

    MoveTurnRobot(ip,port)
    rospy.spin()
