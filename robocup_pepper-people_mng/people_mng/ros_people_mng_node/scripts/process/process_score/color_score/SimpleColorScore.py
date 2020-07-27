__author__ = 'Jacques Saraydaryan'

import rospy
from AbstractColorScore import AbstractColorScore


class SimpleColorScore(AbstractColorScore):

    def __init__(self):
        pass



    def color_score(self, people_color_list, tracked_people, KEY):
        """
        Compute distance between 2 HSV color
        Need to take into account S And V ? to investigate
        :param people_color_list: current people color
        :param tracked_people: registred tracked
        :param KEY: color KEY
        :return: distance between color [0,1]
        """
        tracked_people_color_list=tracked_people.getColorList(KEY)

        if len(people_color_list) == 0 or len(tracked_people_color_list) == 0:
            return 0

        people_hsv = self.get_max_hsv_color(people_color_list)
        tracked_people_hsv = self.get_max_hsv_color(tracked_people_color_list)

        # https://stackoverflow.com/questions/35113979/calculate-distance-between-colors-in-hsv-space
        dh = min(abs(people_hsv[0] - tracked_people_hsv[0]), 360 - abs(people_hsv[0] - tracked_people_hsv[0])) / float(
            360)
        # ds = abs(people_hsv[1] - tracked_people_hsv[1])/float(100)
        # dv = abs(people_hsv[2] - tracked_people_hsv[2]) / float(255)

        # distance = math.sqrt(dh * dh + ds * ds + dv * dv)
        return 1 - dh

    def update_color_score_content(self,tracked_people_id_to_remove_list):
        pass

    def add_color_to_tracked(self, people_color_list, tracked_people, KEY):
        pass