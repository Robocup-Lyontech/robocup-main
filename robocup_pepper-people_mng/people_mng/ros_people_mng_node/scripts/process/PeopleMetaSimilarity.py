#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'

import math
import rospy
from PersonMetaInfo import PersonMetaInfo
from process_score.face_score.SimpleFaceScore import SimpleFaceScore
from process_score.face_score.MemoryFaceScore import MemoryFaceScore

from process_score.color_score.SimpleColorScore import SimpleColorScore
from process_score.color_score.AverageColorScore import AverageColorScore
from process_score.pose_score.SimplePoseScore import SimplePoseScore
from process_score.pose_score.NoPoseScore import NoPoseScore



class PeopleMetaSimilarity:
    PROCESS_FACE_SIMPLE_SCORE="PROCESS_FACE_SIMPLE_SCORE"
    PROCESS_FACE_MEMORY_SCORE = "PROCESS_FACE_MEMORY_SCORE"

    PROCESS_COLOR_SIMPLE_SCORE = "PROCESS_COLOR_SIMPLE_SCORE"
    PROCESS_COLOR_AVG_SCORE = "PROCESS_COLOR_AVG_SCORE"

    PROCESS_POSE_NONE_SCORE = "PROCESS_POSE_NONE_SCORE"
    PROCESS_POSE_SIMPLE_SCORE = "PROCESS_POSE_SIMPLE_SCORE"

    CURRENT_FACE_SCORE = "PROCESS_FACE_MEMORY_SCORE"
    CURRENT_COLOR_SCORE = "PROCESS_COLOR_AVG_SCORE"
    CURRENT_POSE_SCORE = "PROCESS_POSE_SIMPLE_SCORE"

    WEIGHT_FACE_SCORE = 10
    WEIGHT_COLOR_SCORE = 3
    WEIGHT_POSE_SCORE = 5

    def __init__(self):
        self.configure()
        rospy.loginfo("TRACKER SIMILARITY---- FACE-SCORE:" + self.CURRENT_FACE_SCORE)
        rospy.loginfo("TRACKER SIMILARITY---- COLOR-SCORE:" + self.CURRENT_COLOR_SCORE)
        rospy.loginfo("TRACKER SIMILARITY---- POSE-SCORE:" + self.CURRENT_POSE_SCORE)

    def configure(self):
        """
        Configuration
        """
        #Define all the possible score functions
        self.processScoreMap={}
        self.processScoreMap[self.PROCESS_FACE_SIMPLE_SCORE] = SimpleFaceScore()
        self.processScoreMap[self.PROCESS_FACE_MEMORY_SCORE] = MemoryFaceScore()
        self.processScoreMap[self.PROCESS_COLOR_SIMPLE_SCORE] = SimpleColorScore()
        self.processScoreMap[self.PROCESS_COLOR_AVG_SCORE] = AverageColorScore()
        self.processScoreMap[self.PROCESS_POSE_NONE_SCORE] = NoPoseScore()
        self.processScoreMap[self.PROCESS_POSE_SIMPLE_SCORE] = SimplePoseScore()

    def evaluate_people(self, people, tracked_people, new_pose):
        """
        Evaluate the score between current people and registered tracked people
        Score include Face score, color score (HSV) and pose likelihood
        :param people: current people (ros_people_mng_msgs/PeopleMetaInfo)
        :param tracked_people: registered tracked people (ros_people_mng_msgs/PeopleMetaInfo)
        :param new_pose: current people pose
        :return: global score of likelihood [0,1]
        """
        #Score of the face with the parametized score function
        score_face = self.processScoreMap[self.CURRENT_FACE_SCORE].face_score(people, tracked_people)
        #Score of the shirt function with the parametized score function
        score_color_shirt = self.processScoreMap[self.CURRENT_COLOR_SCORE].color_score(people.details.shirtColorList,tracked_people, PersonMetaInfo.SHIRT_RECT)
        #Score of the trouser function with the parametized score function
        score_color_trouser = self.processScoreMap[self.CURRENT_COLOR_SCORE].color_score(people.details.trouserColorList, tracked_people, PersonMetaInfo.TROUSER_RECT)
        #Score of the people position with the parametized score function
        score_pose = self.processScoreMap[self.CURRENT_POSE_SCORE].pose_score(people, tracked_people, new_pose)
        #Final score
        rospy.logdebug("----SCORE:  Face:" + str(score_face) + ", COLOR SHIRT:" + str(score_color_shirt) + ", COLOR_TROUSER:" + str(score_color_trouser) + ", POSE:"+str(score_pose))
        final_score = (
                       self.WEIGHT_FACE_SCORE * score_face + self.WEIGHT_COLOR_SCORE * score_color_shirt +
                       self.WEIGHT_COLOR_SCORE * score_color_trouser + self.WEIGHT_POSE_SCORE * score_pose
               ) / (
                       self.WEIGHT_FACE_SCORE + self.WEIGHT_COLOR_SCORE * 2 + self.WEIGHT_POSE_SCORE
               )
        return final_score, score_face, score_color_shirt, score_color_trouser, score_pose

    def update_process_score(self, tracked_people_id_to_remove_list):
        self.processScoreMap[self.CURRENT_FACE_SCORE].update_face_score_content(tracked_people_id_to_remove_list)
        self.processScoreMap[self.CURRENT_COLOR_SCORE].update_color_score_content(tracked_people_id_to_remove_list)
        self.processScoreMap[self.CURRENT_POSE_SCORE].update_pose_score_content()


    def update_tracked_people(self,people, tracked_people):
        self.processScoreMap[self.CURRENT_FACE_SCORE].add_face_to_tracked(people, tracked_people)
        self.processScoreMap[self.CURRENT_COLOR_SCORE].add_color_to_tracked(
                                                                        people.details.shirtColorList,
                                                                        tracked_people, PersonMetaInfo.SHIRT_RECT
        )
        self.processScoreMap[self.CURRENT_COLOR_SCORE].add_color_to_tracked(
                                                                        people.details.trouserColorList,
                                                                        tracked_people, PersonMetaInfo.TROUSER_RECT
        )
