__author__ = 'Jacques Saraydaryan'

import rospy
from AbstractFaceScore import AbstractFaceScore

class SimpleFaceScore(AbstractFaceScore):


    def __init__(self):
        pass

    def face_score(self, people, tracked_people):
        """
        Compute Face score
        Currently only check if same id and return % of detected face
        :param people: current people
        :param tracked_people: registered tracked people
        :return: 0 is face label do not match, % of face detection otherwise
        """
        if people.label_id == tracked_people.label_id:
            if people.label_id != "None":
                rospy.loginfo("TRACKER: score to label:" + people.label_id)
            # if tracked_people.label_score < people.label_score:
            #    tracked_people.label_score=people.label_score
            return people.label_score
        return 0

    def update_face_score_content(self, tracked_people_id_to_remove_list):
        """
        NOTING TO DO, in current mode no additional update is needed
        :return:
        """
        pass

    def add_face_to_tracked(self, people, tracked_people):
        """
        NOTING TO DO, in current mode no update needed
        :param people:
        :param tracked_people:
        :return:
        """
        pass
