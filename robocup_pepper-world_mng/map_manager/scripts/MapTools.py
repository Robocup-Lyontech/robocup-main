#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
from rospy_message_converter import message_converter, json_message_converter
from robocup_msgs.msg import InterestPoint #, Order, OrderInterest
from geometry_msgs.msg import Pose,PoseStamped
from map_manager.srv import *



class Mt:
    # CONFIG_PATH="/home/astro/ros_ws/src/laptop-youbot/cpe_robotcup/config/interest-points/"
    CONFIG_PATH=""
    _index_label=0


    def __init__(self,conf_path):
        self.CONFIG_PATH=conf_path
        self.configure()
        print

    ####
    #  Set the initial configuration of the current Node
    ####
    def configure(self):
        self._simpleGoal_sub=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simpleGoalcallback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def simpleGoalcallback(self,data):
        itPoint = InterestPoint()
        itPoint.label = "It"+str(self._index_label)
        self._index_label+=1
        itPoint.pose = data.pose
        itPoint.arm_position = 0

        f = open(self.CONFIG_PATH + str(itPoint.label) + '.coord', 'w+')
        json_str = json_message_converter.convert_ros_message_to_json(itPoint)
        f.write(json_str)
        f.close()
        rospy.loginfo('Successfully saved the interestPoint:' + str(json_str))



if __name__ == '__main__':


    ## COMMAND SAMPLE ##
    #
    #  rosrun map_manager MapManager.py _confPath:="/home/astrostudent/evers_ws/conf/ITs"
    #
    ####################
    default_value="/home/xia0ben/pepper_ws/data/world_mng/interest_points/"

    rospy.init_node('map_tools_server')
    config_directory_param=rospy.get_param("~confPath",default_value)
    mt = Mt(config_directory_param)