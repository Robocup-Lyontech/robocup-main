__author__ = 'Jacques Saraydaryan'

import rospy
import threading
from skimage import color
import numpy as np


from AbstractColorScore import AbstractColorScore
from ColorCandidate import ColorCandidate


class AverageColorScore(AbstractColorScore):
    SHIRT_RECT="SHIRT_RECT"
    TROUSER_RECT="TROUSER_RECT"


    def __init__(self):
        self.tracked_color_candidate_map={}
        self.tracked_color_candidate_map_lock = threading.Lock()


    def color_score(self, people_color_list, tracked_people, KEY):
        """
        TODO
        :param tracked_people_color_list: registred tracked people color
        :return: distance between color [0,1]
        """


        if len(people_color_list) == 0 :
            return 0
        try:
            self.tracked_color_candidate_map_lock.acquire()
            #rospy.logwarn("-----------> GET LOCK   COLOR SCORE")
            if self.tracked_color_candidate_map.has_key(tracked_people.id):

                tracked_people_hsv = self.tracked_color_candidate_map[tracked_people.id][KEY].get_avg_color()
                people_hsv = self.get_max_hsv_color(people_color_list)
                # set the average color for the current tracked people
                data = np.zeros(shape=(1, 1, 3), dtype=np.float64)
                normalized_tracked_people_hsv=[0,0,0]
                normalized_tracked_people_hsv[0]=tracked_people_hsv[0]/float(360)
                normalized_tracked_people_hsv[1]=tracked_people_hsv[1]/float(100)
                normalized_tracked_people_hsv[2]=tracked_people_hsv[2]/float(100)

                data[0, 0, :] = normalized_tracked_people_hsv
                rgb = (color.hsv2rgb(data) * 255)[0][0][:]
                tracked_people.colorRGBMap[KEY] = rgb

                # https://stackoverflow.com/questions/35113979/calculate-distance-between-colors-in-hsv-space
                dh = min(abs(people_hsv[0] - tracked_people_hsv[0]),
                         360 - abs(people_hsv[0] - tracked_people_hsv[0])) / float(
                    360)
                # ds = abs(people_hsv[1] - tracked_people_hsv[1])/float(100)
                # dv = abs(people_hsv[2] - tracked_people_hsv[2]) / float(255)

                # distance = math.sqrt(dh * dh + ds * ds + dv * dv)
                return 1 - dh

            else:
                self.tracked_color_candidate_map[tracked_people.id]={}
                self.tracked_color_candidate_map[tracked_people.id][self.SHIRT_RECT]= ColorCandidate()
                self.tracked_color_candidate_map[tracked_people.id][self.TROUSER_RECT] = ColorCandidate()

                return 0
        finally:
            #rospy.logwarn("<----------- RELEASE LOCK   COLOR SCORE")
            self.tracked_color_candidate_map_lock.release()

    def update_color_score_content(self, tracked_people_id_to_remove_list):
        try:
            self.tracked_color_candidate_map_lock.acquire()
            #rospy.logwarn("-----------> GET LOCK   COLOR FORGET")
            # Remove old tracked people
            for key_to_remove in tracked_people_id_to_remove_list:
                if self.tracked_color_candidate_map.has_key(key_to_remove):
                    del self.tracked_color_candidate_map[key_to_remove]
        finally:
            #rospy.logwarn("<----------- RELEASE LOCK   COLOR FORGET")
            self.tracked_color_candidate_map_lock.release()


    def add_color_to_tracked(self,people_color_list, tracked_people, KEY):
        """
        Associate current people face to targeted tracked people
        :param people: current people observation
        :param tracked_people: registered tracked people
        :return: nothing
        """

        try:
            self.tracked_color_candidate_map_lock.acquire()
            people_hsv = self.get_max_hsv_color(people_color_list)
            self.tracked_color_candidate_map[tracked_people.id][KEY].add_color(people_hsv)

        finally:
            self.tracked_color_candidate_map_lock.release()

