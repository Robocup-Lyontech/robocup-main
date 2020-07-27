__author__ = 'Vincent LE DOZE'

import rospy
import math
from AbstractPoseScore import AbstractPoseScore

class NoPoseScore(AbstractPoseScore):

    def __init__(self):
        pass

    def update_pose_score_content(self):
        pass

    def pose_score(self, people, tracked_people, new_pose):
        """
        :param people: current people
        :param tracked_people: evaluated tracked people
        :param x: current people x
        :param y: current people y
        :param z: current people z
        :return: distance score
        """
        return 0.0
