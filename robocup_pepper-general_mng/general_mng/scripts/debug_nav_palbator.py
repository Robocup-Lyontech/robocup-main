#!/usr/bin/env python

import rospy
from meta_lib.LTNavigation import LTNavigation
import math
class Test():

    def __init__(self):
        rospy.init_node("test_LTNavigation")

        self._lt_navigation = LTNavigation()

        rospy.sleep(3)
        self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal", "place_entrance_recep", 90.0)

        rospy.sleep(3)

        self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal", "place_livingRoom", 90.0)

        rospy.sleep(3)

        # self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal", "place_diningRoom", 90.0)

        # rospy.sleep(3)

        self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal", "place_kitchen", 90.0)

        rospy.sleep(3)
        
        self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal", "place_bedroom", 90.0)

        rospy.sleep(3)

        for i in range(0,4):
            rotation_angle = math.pi / 2.0
            rospy.logwarn("ROTATION ANGLE : %s",str(rotation_angle))
            response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)
            rospy.sleep(1)
        

if __name__ == "__main__":

    a=Test()
