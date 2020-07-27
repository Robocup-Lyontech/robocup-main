#!/usr/bin/env python

import rospy
from meta_lib.LTSimulation import LTSimulation

class Test():

    def __init__(self):
        rospy.init_node("test_LTSimulation")

        self._lt_simulation = LTSimulation()

        rospy.sleep(3)

        rospy.loginfo("G1_entrance")
        self._lt_simulation.guest_spawner_for_receptionist("G1_entrance")
        # rospy.sleep(3)
        # rospy.loginfo("G1_before_present")
        # self._lt_simulation.guest_spawner_for_receptionist("G1_before_present")
        # rospy.sleep(3)
        # rospy.loginfo("G1_after_present")
        # self._lt_simulation.guest_spawner_for_receptionist("G1_after_present")
        # rospy.sleep(3)
        # rospy.loginfo("G2_entrance")
        # self._lt_simulation.guest_spawner_for_receptionist("G2_entrance")
        # rospy.sleep(3)
        rospy.loginfo("G2_before_present")
        self._lt_simulation.guest_spawner_for_receptionist("G2_before_present")
        # rospy.sleep(3)

        # self._lt_simulation.reset_guests_for_receptionist()

if __name__ == "__main__":

    a=Test()

