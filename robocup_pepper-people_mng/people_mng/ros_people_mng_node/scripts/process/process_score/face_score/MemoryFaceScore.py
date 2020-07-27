__author__ = 'Jacques Saraydaryan'

import rospy
import math
import time
import threading
from AbstractFaceScore import AbstractFaceScore
from FaceCandidate import FaceCandidate

class MemoryFaceScore(AbstractFaceScore):
    NONE_LABEL="None"
    UNKNOWN_LABEL="Unknown"
    FORGET_WEIGHT_THRESHOLD = 10
    FORGET_TIME_THRESHOLD = 30
    MIN_FACE_PER_TRACKED_PEOPLE = 3



    def __init__(self):
        self.tracked_people_face_candidate_map={}
        self.tracked_people_face_candidate_map_lock = threading.Lock()


    def face_score(self, people, tracked_people):
        """
        Compute Face score
        TODO
        :param people: current people
        :param tracked_people: registered tracked people
        :return: TODO
        """
        result_to_return=0

        # people.label_id == None  return 0
        if people.label_id == self.NONE_LABEL:
            return 0


        try:
            self.tracked_people_face_candidate_map_lock.acquire()
            # check if the current tracked people has got face candidate
            if self.tracked_people_face_candidate_map.has_key(tracked_people.id):
                # check if face id already registered for the current tracked people
                if self.tracked_people_face_candidate_map[tracked_people.id].has_key(people.label_id):
                    f_candidate=self.tracked_people_face_candidate_map[tracked_people.id][people.label_id]
                    #f_candidate.inc_weight(1)
                    result_to_return =  self.compute_face_score(people.label_id,tracked_people)
                else:
                    # if face bot currently associated to the tracked people return 0
                    result_to_return = 0

            else:
                # create a dictionary of face candidate for the current tracked people
                self.tracked_people_face_candidate_map[tracked_people.id]={}
                #rospy.loginfo("      CREATE NEW TRACKED !!!!: tracked:" + tracked_people.id + ", face added:" + people.label_id)
                result_to_return = 0

            max_face_key = self.get_max_face_to_tracked(tracked_people.id)
            if self.tracked_people_face_candidate_map[tracked_people.id].has_key(max_face_key):
                tracked_people.label_id = self.tracked_people_face_candidate_map[tracked_people.id][max_face_key].label
                tracked_people.label_score = self.tracked_people_face_candidate_map[tracked_people.id][max_face_key].max_score
                rospy.logdebug("      MAX LABEL: tracked:" + tracked_people.id + ", face added:" + tracked_people.label_id)
            return result_to_return;
        finally:
            self.tracked_people_face_candidate_map_lock.release()

    def update_face_score_content(self, tracked_people_id_to_remove_list):
        """
        TODO
        :return:
        """

        try:
            self.tracked_people_face_candidate_map_lock.acquire()

            # Remove old tracked people
            for key_to_remove in tracked_people_id_to_remove_list:
                if self.tracked_people_face_candidate_map.has_key(key_to_remove):
                    del self.tracked_people_face_candidate_map[key_to_remove]


            # Remove old face associated to tracked people
            for tracked_key in self.tracked_people_face_candidate_map.keys():
                index_to_remove = []
                #rospy.loginfo("------------- TRACKED:" + str(tracked_key)+"------------- ")
                for face_key in self.tracked_people_face_candidate_map[tracked_key].keys():
                    #rospy.loginfo("      FACE:" + str(face_key) + ", weight:"+str(self.tracked_people_face_candidate_map[tracked_key][face_key].weight)+",max_score: "+str(self.tracked_people_face_candidate_map[tracked_key][face_key].max_score))
                    elapsed_time = time.time() - self.tracked_people_face_candidate_map[tracked_key][face_key].last_update_time

                    if self.forget_function(elapsed_time, self.tracked_people_face_candidate_map[tracked_key][face_key].weight, self.FORGET_WEIGHT_THRESHOLD, self.FORGET_TIME_THRESHOLD) == 0:
                        index_to_remove.append(face_key)

                max_value_to_remove = len(self.tracked_people_face_candidate_map[tracked_key]) - self.MIN_FACE_PER_TRACKED_PEOPLE
                index = 0
                for i in index_to_remove:
                    if index < max_value_to_remove:
                        # remove associated face to current tracked people
                        del self.tracked_people_face_candidate_map[tracked_key][i]
                    else:
                        break
                    index = index + 1

        finally:
            self.tracked_people_face_candidate_map_lock.release()


    def compute_face_score(self, current_label, tracked_people):
        """
        Compute the score regarding the face information
        :param current_label: current face label
        :param tracked_people: registered tracked people
        :return: current detected face weight (associated to tracked people) / sum of all face weight associated to the tracked people
        """
        face_map = self.tracked_people_face_candidate_map[tracked_people.id]
        weight_sum=0
        for face_key in face_map.keys():
            weight_sum = weight_sum + face_map[face_key].weight

        return face_map[current_label].weight / float(weight_sum)

    def forget_function(self,t,weight,weigth_threshold,time_threshold):
        value = ( weigth_threshold * -(1 / min(float(weigth_threshold), float(weight))) ) * t + float(time_threshold)
        if value < 0:
            return 0
        else:
            return 1 + math.log( value )

    def get_max_face_to_tracked(self, tracked_key):
        max_weight=0
        max_face_key=""
        for face_key in self.tracked_people_face_candidate_map[tracked_key].keys():
            if self.tracked_people_face_candidate_map[tracked_key][face_key].weight > max_weight:
                max_weight = self.tracked_people_face_candidate_map[tracked_key][face_key].weight
                max_face_key = face_key
        return max_face_key


    def add_face_to_tracked(self,people,tracked_people):
        """
        Associate current people face to targeted tracked people
        :param people: current people observation
        :param tracked_people: registered tracked people
        :return: nothing
        """

        try:
            self.tracked_people_face_candidate_map_lock.acquire()
            if not self.tracked_people_face_candidate_map.has_key(tracked_people.id):
                #rospy.logwarn("TRACKER FACE MULTI: target no existing tracked people in face multi (add_face_to_tracked function) :"+tracked_people.id)
                return
            if self.tracked_people_face_candidate_map[tracked_people.id].has_key(people.label_id):
                #rospy.loginfo(
                #    "      UPDATE:    CREATE Existing FACE: tracked:" + tracked_people.id + ", face :" + people.label_id)
                f_candidate = self.tracked_people_face_candidate_map[tracked_people.id][people.label_id]
                f_candidate.inc_weight(1)
            else:
                if people.label_id != self.NONE_LABEL and people.label_id != self.UNKNOWN_LABEL:
                    #   rospy.loginfo(
                    #    "      UPDATE:    CREATE NEW FACE: tracked:" + tracked_people.id + ", face added:" + people.label_id)
                    f_candidate = FaceCandidate()
                    f_candidate.label = people.label_id
                    f_candidate.max_score = people.label_score
                    f_candidate.weight = 1
                    self.tracked_people_face_candidate_map[tracked_people.id][people.label_id] = f_candidate


        finally:
            self.tracked_people_face_candidate_map_lock.release()
