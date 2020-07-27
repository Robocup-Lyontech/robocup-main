#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'

import uuid
import time
import math
import rospy
from PeopleMetaSimilarity import PeopleMetaSimilarity
from tools.StatMng import StatMng
from TrackedPersonMetaInfo import TrackedPersonMetaInfo
from PersonMetaInfo import PersonMetaInfo
import threading


class PeopleMetaTrackerMng:
    SCORE_THRESHOLD = 0.2 #0.3
    FORGETTING_PERIOD = 1
    isForgettingThreadEnd = False
    ACTION_CREATE = "CREATE"
    ACTION_UPDATE = "UPDATE"
    ACTION_DELETE = "DELETE"

    def __init__(self, stat_folder):
        self.trackedPeopleMap = {}
        self.forget_param_map = {}
        self.peopleSimilarity = PeopleMetaSimilarity()
        self.statMng = StatMng(stat_folder)
        self.tracked_people_map_lock = threading.Lock()
        self.forgetting_thread = threading.Thread(target=self.forgetting_callback, args=(self.FORGETTING_PERIOD,))
        self.configure()

        self.forgetting_thread.start()
        rospy.loginfo("TRACKER---- READY TO TRACK")

    def configure(self):
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_NONE]=(10,30)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_OPERATOR] = (10, 30)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_PERSON_OF_INTEREST] = (10, 30)

    def track_people(self, peopleList):

        for people in peopleList:
            new_pose = self.get_people_pose(people.pose)

            max_score = [0, 0, 0, 0, 0]
            max_id = ""

            ## protected section on tracked people map
            self.tracked_people_map_lock.acquire()
            for tracked_people_key in self.trackedPeopleMap:

                score = self.peopleSimilarity.evaluate_people\
                        (
                            people,
                            self.trackedPeopleMap[tracked_people_key],
                            new_pose
                        )
                rospy.logdebug("---- CURRENT TOTAL SCORE:" + str(score) + ", People_label:" + people.label_id+", TRACKED PEOPLE ID:" + str(tracked_people_key))
                if score[0] > max_score[0]:
                    max_score = score
                    max_id = tracked_people_key
            self.tracked_people_map_lock.release()
            ## End of protected section

            rospy.logdebug("MAX SCORE:" + str( max_score) + ", People_label:" + people.label_id + ", TRACKED PEOPLE ID:" + str(max_id))
            if max_score[0] < self.SCORE_THRESHOLD:


                # According to the score update existing trackedPeople or create a new one
                new_tracked_people = TrackedPersonMetaInfo(
                    str(uuid.uuid1()),
                    people.label_id,
                    people.label_score,
                    people.details.boundingBox, people.details.shirtRect,
                    people.details.trouserRect, people.details.shirtColorList,
                    people.details.trouserColorList
                )
                new_tracked_people.setMainColor(PersonMetaInfo.SHIRT_RECT, people.shirt_color_name, self.getMaxColor(people.details.shirtColorList))
                new_tracked_people.setMainColor(PersonMetaInfo.TROUSER_RECT, people.trouser_color_name, self.getMaxColor(people.details.trouserColorList))
                new_tracked_people.setPose(new_pose)
                new_tracked_people.posture=people.posture
                new_tracked_people.handPosture = people.handPosture
                rospy.logdebug("CREATE NEW TRACKED PEOPLE:" + str(new_tracked_people.id))
                self.statMng.update_stat(self.ACTION_CREATE, new_tracked_people.id, 0, 0, 0, 0, 0)
                self.addTrackedPeople(new_tracked_people)

            else:
                rospy.logdebug("UPDATE EXISTING TRACKED PEOPLE:" + str(max_id))
                self.statMng.update_stat(self.ACTION_CREATE, max_id, max_score[0], max_score[1], max_score[2], max_score[3], max_score[4])
                self.updateTrackedPeople(max_id, people, new_pose, max_score, True)
        return self.getTrackedPeopleList()

    def track_people_best_per_tracked(self, peopleList):
        score_per_tracked_map={}
        ## protected section on tracked people map
        self.tracked_people_map_lock.acquire()
        for tracked_people_key in self.trackedPeopleMap:
            score_per_tracked_map[tracked_people_key]=[]

        max_score_per_people={}
        new_pose_per_people = {}
        #compute 1 score per people per tracked people
        for people in peopleList:
            new_pose = self.get_people_pose(people.pose)
            new_pose_per_people[people.id]=new_pose
            max_score = [0, 0, 0, 0, 0]

            for tracked_people_key in self.trackedPeopleMap:
                score = self.peopleSimilarity.evaluate_people \
                        (
                        people,
                        self.trackedPeopleMap[tracked_people_key],
                        new_pose
                    )
                rospy.logdebug("---- CURRENT TOTAL SCORE:" + str(
                    score) + ", People_label:" + people.label_id + ", TRACKED PEOPLE ID:" + str(tracked_people_key))
                self.statMng.update_stat_versus(people.id, tracked_people_key, score[0], score[1], score[2],
                                                score[3], score[4])

                if score[0] > max_score[0]:
                    max_score = score

                score_per_tracked_map[tracked_people_key].append((people,score))
            max_score_per_people[people.id]=max_score


        # create new cluster if needed
        affected_people_id_list = []

        for people in peopleList:
            if max_score_per_people[people.id][0] < self.SCORE_THRESHOLD:
                affected_people_id_list.append(people.id)

                # According to the score update existing trackedPeople or create a new one
                new_tracked_people = TrackedPersonMetaInfo(
                    str(uuid.uuid1()),
                    people.label_id,
                    people.label_score,
                    people.details.boundingBox, people.details.shirtRect,
                    people.details.trouserRect, people.details.shirtColorList,
                    people.details.trouserColorList
                )
                new_tracked_people.setMainColor(PersonMetaInfo.SHIRT_RECT, people.shirt_color_name,
                                                self.getMaxColor(people.details.shirtColorList))
                new_tracked_people.setMainColor(PersonMetaInfo.TROUSER_RECT, people.trouser_color_name,
                                                self.getMaxColor(people.details.trouserColorList))
                new_tracked_people.setPose(new_pose_per_people[people.id])
                new_tracked_people.posture = people.posture
                new_tracked_people.handPosture = people.handPosture
                rospy.logdebug("CREATE NEW TRACKED PEOPLE:" + str(new_tracked_people.id))
                self.statMng.update_stat(self.ACTION_CREATE,people.id, new_tracked_people.id, 0, 0, 0, 0, 0)
                self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people
                score_per_tracked_map[new_tracked_people.id]=[]

        # affect best score to tracked people need to order per max score
        # ordered = sorted(score_per_tracked_map.items(), key=lambda attributes: attributes[1][1])
        # =sorted(, key=lambda t: t[1][1])
        # print(ordered)
        order_tracked_key_list=self.order_track_per_score(score_per_tracked_map)
        for tracked_people_key in order_tracked_key_list:
            max_score=[0, 0, 0, 0, 0]
            max_people=0
            for p,s in score_per_tracked_map[tracked_people_key]:
                if(s[0]>max_score[0] and  p.id not in affected_people_id_list):
                    max_score=s
                    max_people=p
            #check if the selected people has a score below threshold or not
            if max_score[0] > self.SCORE_THRESHOLD:
                #check if people not already affected
                if max_people.id not in affected_people_id_list:
                    affected_people_id_list.append(max_people.id)
                    rospy.logdebug("UPDATE EXISTING TRACKED PEOPLE:" + str(max_people.id))
                    self.statMng.update_stat(self.ACTION_UPDATE,max_people.id, tracked_people_key, max_score[0], max_score[1], max_score[2], max_score[3], max_score[4])

                    # # CAUTION
                    # # # for all tracked only for stat
                    # for t_tmp in score_per_tracked_map:
                    #         for p_tmp,s_tmp in score_per_tracked_map[t_tmp]:
                    #             if p_tmp.id == p.id :
                    #                 self.statMng.update_stat_versus(tracked_people_key, t_tmp, s_tmp[0], s_tmp[1],
                    #                                                 s_tmp[2], s_tmp[3], s_tmp[4])

                    self.updateTrackedPeople(tracked_people_key, max_people, new_pose_per_people[max_people.id], max_score,False)
        # for all people not affected create a new tracked people
        for people in peopleList:
            if people.id not in affected_people_id_list:
                # According to the score update existing trackedPeople or create a new one
                new_tracked_people = TrackedPersonMetaInfo(
                    str(uuid.uuid1()),
                    people.label_id,
                    people.label_score,
                    people.details.boundingBox, people.details.shirtRect,
                    people.details.trouserRect, people.details.shirtColorList,
                    people.details.trouserColorList
                )
                new_tracked_people.setMainColor(PersonMetaInfo.SHIRT_RECT, people.shirt_color_name,
                                                self.getMaxColor(people.details.shirtColorList))
                new_tracked_people.setMainColor(PersonMetaInfo.TROUSER_RECT, people.trouser_color_name,
                                                self.getMaxColor(people.details.trouserColorList))
                new_tracked_people.setPose(new_pose_per_people[people.id])
                new_tracked_people.posture = people.posture
                new_tracked_people.handPosture = people.handPosture
                rospy.logdebug("CREATE NEW TRACKED PEOPLE:" + str(new_tracked_people.id))
                self.statMng.update_stat(self.ACTION_CREATE,people.id, new_tracked_people.id, 0, 0, 0, 0, 0)
                self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people

        self.tracked_people_map_lock.release()
        return self.getTrackedPeopleList()



    def order_track_per_score(self,score_per_tracked_map):
        tracked_max_score_map={}
        for tracked_key in score_per_tracked_map:
            max_score=0
            for p,s in score_per_tracked_map[tracked_key]:
                if s[0]> max_score:
                    max_score=s[0]
            tracked_max_score_map[tracked_key]=max_score

        return sorted(tracked_max_score_map, key=lambda t: t[1], reverse=True)




    def addTrackedPeople(self, new_tracked_people):
        self.tracked_people_map_lock.acquire()
        self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people
        self.tracked_people_map_lock.release()

    def updateTrackedPeople(self, id , people, new_pose, score, isLock):
        if isLock:
            self.tracked_people_map_lock.acquire()
        if self.trackedPeopleMap[id].label_id == 'None':
            self.trackedPeopleMap[id].label_id = people.label_id
        self.trackedPeopleMap[id].setPose(new_pose)
        self.trackedPeopleMap[id].incWeight()
        self.trackedPeopleMap[id].setBoundingBox(PersonMetaInfo.PERSON_RECT,people.details.boundingBox)
        self.trackedPeopleMap[id].last_score=score[0]
        self.trackedPeopleMap[id].last_score_face=score[1]
        self.trackedPeopleMap[id].last_score_c_s=score[2]
        self.trackedPeopleMap[id].last_score_c_t=score[3]
        self.trackedPeopleMap[id].last_score_pose=score[4]
        self.trackedPeopleMap[id].posture=people.posture
        self.trackedPeopleMap[id].handPosture = people.handPosture

        # update additional information relative to tracked people
        self.peopleSimilarity.update_tracked_people(people,self.trackedPeopleMap[id])
        if isLock:
            self.tracked_people_map_lock.release()

    def getTrackedPeopleList(self):
        self.tracked_people_map_lock.acquire()
        result=self.trackedPeopleMap.values()
        self.tracked_people_map_lock.release()
        return result

    # def get_tracked_people(self, id):
    #     self.tracked_people_map_lock.acquire()
    #     current_tracked = self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()
    #     return current_tracked

    # def remove_tracked_people(self,id):
    #     self.tracked_people_map_lock.acquire()
    #     del self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()

    def get_people_pose(self, pose):
        # TODO Make tf transform from curren people camera pose and map people map pose
        return pose

    def stop_forgetting_function(self):
        self.isForgettingThreadEnd=True
        self.forgetting_thread.join(5)

    def forgetting_callback(self,period):

            while not self.isForgettingThreadEnd:
                self.tracked_people_map_lock.acquire()
                id_to_remove = []
                for tracked_people_key in self.trackedPeopleMap:
                    #get current tracked people
                    current_tracked_people = self.trackedPeopleMap[tracked_people_key]
                    #compute elasped time since last update
                    elapsed_time = time.time()-current_tracked_people.last_update_time
                    # get forget parameters according people role
                    (weigth_threshold,time_threshold) = self.forget_param_map[current_tracked_people.role]
                    # compute forget function
                    forget_result=  self.forget_function(elapsed_time,current_tracked_people.weight,weigth_threshold,time_threshold)
                    self.trackedPeopleMap[tracked_people_key].ttl = forget_result
                    if forget_result <= 0:
                        # remove current tracked people
                        id_to_remove.append(tracked_people_key)

                for id in id_to_remove:
                    try:
                        self.statMng.update_stat(self.ACTION_DELETE,'NONE', str(id), 0, 0, 0, 0, 0)
                        del self.trackedPeopleMap[str(id)]
                    except KeyError as e:
                        rospy.logwarn("unable to del tracked people e:"+str(e))

                # ask for update of process score function
                self.peopleSimilarity.update_process_score(id_to_remove)

                self.tracked_people_map_lock.release()

                time.sleep(period)

    def forget_function(self,t,weight,weigth_threshold,time_threshold):
        value = ( weigth_threshold * -(1 / min(float(weigth_threshold), float(weight))) ) * t + float(time_threshold)
        if value < 0:
            return 0
        else:
            return 1 + math.log( value )



    def getMaxColor(self,colorList):
        rgb=[]
        max=0.0
        for color in colorList:
            if color.percentage_of_img > max:
                max=color.percentage_of_img
                rgb=color.rgb
        return rgb