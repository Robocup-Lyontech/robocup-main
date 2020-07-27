__author__ = 'Jacques Saraydaryan'

import rospy
import math
from AbstractPoseScore import AbstractPoseScore

class SimplePoseScore(AbstractPoseScore):
    DISTANCE_FONCTION_THRESHOLD = 2

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
        distance = math.sqrt(pow(tracked_people.pose.position.x - new_pose.position.x, 2) + pow(
            tracked_people.pose.position.y - new_pose.position.y, 2) + pow(
            tracked_people.pose.position.z - new_pose.position.z, 2))

        result = math.exp(distance * float(-1) / float(self.DISTANCE_FONCTION_THRESHOLD))

        # TODO to complete according to last registered pose and time elapsed,

        return result