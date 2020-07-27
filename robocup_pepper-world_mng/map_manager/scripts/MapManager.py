#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
from rospy_message_converter import message_converter, json_message_converter
from robocup_msgs.msg import InterestPoint #, Order, OrderInterest
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from map_manager.srv import *
from tf import TransformListener
import tf
import thread
import time

import os

######### Command to Test
##  rosservice call /save_InterestPoint '{"itP":{ "label": "A_sim",  "pose":{ "position":{  "x": 7.33,  "y": 7.18,  "z": 0.0}, "orientation":{ "x": 0.0,  "y": 0.0,  "z": 0.0,  "w": 1.0}}, "arm_position": 0}}'
##  rosservice call /save_InterestPoint '{"itP":{ "label": "B_sim",  "pose":{ "position":{  "x": 1.0,  "y": 1.0,  "z": 0.0}, "orientation":{ "x": 0.0,  "y": 0.0,  "z": 0.0,  "w": 1.0}}, "arm_position": 0}}'
##  rosservice call /save_InterestPoint '{"itP":{ "label": "F_sim",  "pose":{ "position":{  "x": 3.56,  "y": 3.52,  "z": 0.0}, "orientation":{ "x": 0.0,  "y": 0.0,  "z": 0.0,  "w": 1.0}}, "arm_position": 0}}'
#########


class Mm:
    
    # CONFIG_PATH="/home/astro/ros_ws/src/laptop-youbot/cpe_robotcup/config/interest-points/"
    CONFIG_PATH=""
    _mapIP_Position = {}
    _mapOrient_Position = {}
    _loadPoint_service = ""
    _savePoint_service = ""
    _saveitPBaseLink_service = ""
    _getPoint_service = ""
    _saveOrient_service = ""
    _getOrient_service = ""
    _activateTF_service = ""
    _tflistener = ""
    _tfPublisherRunning=True
    _reloadItAfterPublishingTf=False
    _broadcastTfPeriod = 2

    def __init__(self,conf_path):
        self.CONFIG_PATH=conf_path

        self.configure()

    ####
    #  Set the initial configuration of the current Node
    ####
    def configure(self):
        self._loadPoint_service = rospy.Service('load_InterestPoint', Empty, self.loadInterestPoint)
        self._savePoint_service = rospy.Service('save_InterestPoint', saveitP_service, self.saveInterestPoint)
        self._getPoint_service = rospy.Service('get_InterestPoint', getitP_service, self.getInterestPoint)
        self._saveitPBaseLink_service = rospy.Service('save_BaseLinkInterestPoint', saveitP_base_link_service, self.saveBaseLinkInterestPoint)
        self._activateTF_service = rospy.Service('activate_InterestPointTF', activateTF_service, self.activeTFProvider)

        self._tflistener = TransformListener()

        #self.loadInterestPoint()

        #start publishing It Tf
        thread.start_new_thread(self.publishInterestPointTf,())

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    ####
    #  Get interest points from memory
    ####
    def getInterestPoint(self, req):
        rospy.loginfo("Reloading itP list")
        self.loadInterestPoint()
        rospy.loginfo("in the get interest point service. itP name = %s", req.itP_name)
        return self._mapIP_Position[str(req.itP_name)] # return of type InterestPoint

    ####
    #  Save interest points to config file
    ####
    def saveInterestPoint(self, req):
        f = open(self.CONFIG_PATH + str(req.itP.label) + '.coord', 'w+')
        json_str = json_message_converter.convert_ros_message_to_json(req.itP)
        f.write(json_str)
        f.close()
        rospy.loginfo('Successfully saved the interestPoint:' + str(json_str))
        self._mapIP_Position[str(req.itP.label)] = req.itP
        return True


    ####
    #  Save current robot position to config file (format interestPoint)
    ####
    def saveBaseLinkInterestPoint(self, req):
        # transform = StampedTransform()
        # self._tflistener.lookupTransform("base_link", "/map", rospy.Time.now(), transform)
        # self._tflistener.waitForTransform("base_link", "/map", rospy.Time.now(), rospy.Duration
        now = rospy.Time(0)
        self._tflistener.waitForTransform("/map", "/base_link", now, rospy.Duration(10.0))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]
        robotPose.orientation.x = rot[0]
        robotPose.orientation.y = rot[1]
        robotPose.orientation.z = rot[2]
        robotPose.orientation.w = rot[3]

        itPoint = InterestPoint()
        itPoint.label = req.label
        itPoint.pose = robotPose
        itPoint.arm_position = 0

        f = open(self.CONFIG_PATH + str(req.label) + '.coord', 'w+')
        json_str = json_message_converter.convert_ros_message_to_json(itPoint)
        f.write(json_str)
        f.close()
        rospy.loginfo('Successful save ot the interestPoint' + str(json_str))
        return True

    ####
    #  Load interest points from config file
    ####
    def loadInterestPoint(self):
        dirs = os.listdir(self.CONFIG_PATH)
        #Reload IP in the map
        #self._mapIP_Position={}
        # This would print all the files and directories
        for fileName in dirs:

            f = open(self.CONFIG_PATH + fileName, 'r')
            json_str = f.read()
            interestPoint = json_message_converter.convert_json_to_ros_message('robocup_msgs/InterestPoint', json_str)
            #save the interest point to the dictionary (in memory)
            self._mapIP_Position[str(interestPoint.label)] = interestPoint
            f.close()



            # if fileName.find('.coord', 0, len(fileName)) != -1:
            #     f = open(self.CONFIG_PATH + fileName, 'r')
            #     json_str = f.read()
            #     interestPoint = json_message_converter.convert_json_to_ros_message('robot_common_msg/InterestPoint', json_str)
            #     #save the interest point to the dictionary (in memory)
            #     self._mapIP_Position[str(interestPoint.label)] = interestPoint
            #     f.close()
            # elif fileName.find('.orient', 0, len(fileName)) != -1:
            #     f = open(self.CONFIG_PATH + fileName, 'r')
            #     json_str = f.read()
            #     orient = json_message_converter.convert_json_to_ros_message('robot_common_msg/InterestPoint', json_str)
            #     #save the orient to the dictionary (in memory)
            #     self._mapOrient_Prospy_message_converterosition[str(orient.label)] = orient
            #     f.close()

        # Display current map
        rospy.loginfo("Interest points loaded")
        for k, v in self._mapIP_Position.iteritems():
            rospy.loginfo("id:" + str(k) + ",interestPoint:" + str(v))

        # rospy.loginfo("Orientation loaded")
        # for k, v in self._mapOrient_Position.iteritems():
        #     rospy.loginfo("id:" + str(k) + ",orientation:" + str(v))

    ####
    #  Launch or stop a thread in charge of the diffusion of interest points TF (positions)
    ####
    def activeTFProvider(self,req):
        if(req.isActivated and self._tfPublisherRunning):
            rospy.loginfo('[MAP_MANAGER]: Keep tf broadcast enable')
            return []
        elif (not req.isActivated and self._tfPublisherRunning):
            self._tfPublisherRunning=False
            rospy.loginfo('[MAP_MANAGER]: Disable tf broadcast')
            return []
        elif ( req.isActivated and not self._tfPublisherRunning):
            self._tfPublisherRunning=True
            thread.start_new_thread(self.publishInterestPointTf,())
            rospy.loginfo('[MAP_MANAGER]: Start tf broadcast')
            return []
        if(not req.isActivated and not self._tfPublisherRunning):
            rospy.loginfo('[MAP_MANAGER]: Keep tf broadcast disable')
            return []

    ####
    #  Publishing current interest points TF (positions) and reload every 2s interest points from files
    #  loop until self._tfPublisherRunning = False
    ####
    def publishInterestPointTf(self):
        while(self._tfPublisherRunning and not rospy.is_shutdown()):
            br = tf.TransformBroadcaster()
            for k, v in self._mapIP_Position.iteritems():
                # br.sendTransform((v.pose.position.x, v.pose.position.y, v.pose.position.z),
                #                  (v.pose.orientation.x, v.pose.orientation.y,v.pose.orientation.z,v.pose.orientation.w),
                #                  rospy.Time.now(),
                #                  str(k)+'_TF',
                #                  "map")
                br.sendTransform((v.pose.position.x, v.pose.position.y, v.pose.position.z),
                                 (v.pose.orientation.x, v.pose.orientation.y,v.pose.orientation.z,v.pose.orientation.w),
                                 rospy.Time(0),
                                 str(k)+'_TF',
                                 "map")
            time.sleep(self._broadcastTfPeriod)

            if self._reloadItAfterPublishingTf:
                #reload configuration files
                self.loadInterestPoint()

if __name__ == '__main__':


    ## COMMAND SAMPLE ##
    #
    #  rosrun map_manager MapManager.py _confPath:="/home/astrostudent/evers_ws/conf/ITs"
    #
    ####################
    # default_value="/home/xia0ben/pepper_ws/data/world_mng/interest_points/"
    default_value="./data/world_mng/interest_points/"
    rospy.init_node('map_management_server')
    config_directory_param=rospy.get_param("~confPath",default_value)
    mm = Mm(config_directory_param)
