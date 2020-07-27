#!/usr/bin/env python
__author__ = 'Jacques Saraydaryan'

import matplotlib.pyplot as plt
import numpy as np
import os
import time
import rospy
from operator import itemgetter

class StatMng:

    stat_score_matrix = []
    stat_score_matrix_vs_tracked = []
    stat_people_vs_tracked = []
    FILE_PREFIX_A = "PEOPLE_TRACKED_STAT"
    FILE_PREFIX_B = "PEOPLE_TRACKED_STAT_VS_TRACKED"
    FILE_PREFIX_C = "PEOPLE_VS_TRACKED"
    start_time=0

    def __init__(self, save_folder):
        self.save_folder = save_folder
        self.start_time = time.time()


    def update_stat(self, action_type,people_id, tracked_id, general_score, face_score, shirt_color_score, trouser_color_score, pose_score):
        """
        Add stat of a current tracked people
        :param tracked_id: id of the current tracked people
        :param general_score: weigthed sum of score
        :param face_score: score relative to face information
        :param shirt_color_score: score relative to the shirt color information
        :param trouser_color_score: score relative to the trouser color information
        :param pose_score: score relative to pose information
        :param action_type: type of action CREATE vs UPDATE vs DELETE
        """
        time_elapsed = time.time() -self.start_time
        self.stat_score_matrix.append([time_elapsed, action_type, tracked_id,general_score,face_score,shirt_color_score,trouser_color_score,pose_score])
        self.stat_people_vs_tracked.append([time_elapsed, people_id, tracked_id,general_score])

    def update_stat_versus(self, people_id,tracked_id_target, general_score, face_score, shirt_color_score, trouser_color_score, pose_score):
            time_elapsed=time.time() - self.start_time
            self.stat_score_matrix_vs_tracked.append([time_elapsed,people_id,tracked_id_target, general_score, face_score, shirt_color_score, trouser_color_score,
             pose_score])

    def reset_stat(self):
        self.stat_score_matrix=[]

    def save_stat(self):
        file_to_save = self.adjustFileName(self.save_folder,self.FILE_PREFIX_A)
        rospy.loginfo("Save current stat to :" + file_to_save)
        np.save(file_to_save , self.stat_score_matrix)

        file_to_save_versus = self.adjustFileName(self.save_folder, self.FILE_PREFIX_B)
        np.save(file_to_save_versus , self.stat_score_matrix_vs_tracked)

        file_to_save_versus = self.adjustFileName(self.save_folder, self.FILE_PREFIX_C)
        np.save(file_to_save_versus, self.stat_people_vs_tracked)

    def adjustFileName(self, file_path, file_name_prefix):
        if os.path.isfile(file_path+'0-'+ file_name_prefix+".npy"):
            index=1
            while os.path.isfile(file_path+str(index)+'-'+file_name_prefix+".npy"):
                index=index+1
            return file_path+str(index)+'-'+ file_name_prefix
        else:
            return file_path+'0-'+ file_name_prefix



    def load_and_display(self,load_folder, file_name):
        data_matrix = []
        tracked_stat_map={}
        if os.path.isfile(load_folder + file_name):
            data_matrix.append(np.load(load_folder + file_name))
        else:
            rospy.logwarn("Unable to load stat file : "+ str(load_folder + file_name))
            return

        for row in data_matrix[0]:
            tracked_row = []
            # time
            tracked_row.append(float(row[0]))
            # action
            tracked_row.append(row[1])
            # score
            tracked_row.append(float(row[3]))
            # score face
            tracked_row.append(float(row[4]))
            # score shirt color
            tracked_row.append(float(row[5]))
            # score trouser color
            tracked_row.append(float(row[6]))
            # score pose
            tracked_row.append(float(row[7]))

            if tracked_stat_map.has_key(row[2]):
                tracked_stat_map[row[2]].append(tracked_row)
            else:
                tracked_stat_map[row[2]] = []
                tracked_stat_map[row[2]].append(tracked_row)

        fig, axs = plt.subplots(5, 1)
        #plt.subplot(311)
        rospy.loginfo(str(data_matrix))

        # check here --> https://stackoverflow.com/questions/24432470/matplotlib-pyplot-in-real-time
        # key_currnent_index = {}
        #
        # max_index=0
        # for key in tracked_stat_map:
        #     tracked_stat_map[key]=sorted(tracked_stat_map[key], key=itemgetter(0))
        #     key_currnent_index[key]=0
        #     if len(tracked_stat_map[key])>max_index:
        #         max_index=len(tracked_stat_map[key])
        #
        #         # draw the figure so the animations will work
        # fig = plt.gcf()
        # fig.show()
        # fig.canvas.draw()
        #
        # for i in range(0, max_index-1):
        #     for key in tracked_stat_map:
        #         #data_np = np.asarray(tracked_stat_map[key])
        #         idex=key_currnent_index[key]
        #         for p in range(key_currnent_index[key],len(tracked_stat_map[key])):
        #             values=tracked_stat_map[key][p]
        #             idex=idex+1
        #             if values[0]<i:
        #                 plt.plot(values[0],values[2],'ko--', label=key)
        #                 key_currnent_index[key]=idex
        #             else:
        #                 break
        #     # update canvas immediately
        #     plt.xlim([0, 250])
        #     plt.ylim([0, 1])
        #     plt.pause(0.05)
        #     fig.canvas.draw()



        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[0].set_title('General Score')
            p=axs[0].plot(data_np[:,0], data_np[:,2], label=key)
            #axs[0].fill_between(data_np[:,0],0, data_np[:,2],where= data_np[:,2] >0, alpha=0.5)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[2]))
            axs[0].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

            #plt.subplot(312)
        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[1].set_title('Face Score')
            p =axs[1].plot(data_np[:,0], data_np[:,3], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[3]))
            axs[1].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        for key in tracked_stat_map:
            data_np = np.asarray(tracked_stat_map[key])
            axs[2].set_title('Shirt Color Score')
            p =axs[2].plot(data_np[:, 0], data_np[:, 4], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[4]))
            axs[2].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        for key in tracked_stat_map:
            data_np = np.asarray(tracked_stat_map[key])
            axs[3].set_title('Trouser Color Score')
            p =axs[3].plot(data_np[:, 0], data_np[:, 5], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[5]))
            axs[3].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

            #plt.subplot(313)
        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[4].set_title('Pose Score')
            p =axs[4].plot(data_np[:,0], data_np[:,6], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[6]))
            axs[4].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        plt.legend(loc='best')
        plt.show()


    def load_and_display_vs(self,load_folder, file_name):
        data_matrix = []
        tracked_stat_map={}
        if os.path.isfile(load_folder + file_name):
            data_matrix.append(np.load(load_folder + file_name))
        else:
            rospy.logwarn("Unable to load stat file : "+ str(load_folder + file_name))
            return

        tracked_people_list=[]

        for row in data_matrix[0]:
            tracked_row = []
            # time
            tracked_row.append(float(row[0]))
            # score
            tracked_row.append(float(row[3]))
            # score face
            tracked_row.append(float(row[4]))
            # score shirt color
            tracked_row.append(float(row[5]))
            # score trouser color
            tracked_row.append(float(row[6]))
            # score pose
            tracked_row.append(float(row[7]))

            if row[2] not in tracked_people_list:
                tracked_people_list.append(row[2])

            if tracked_stat_map.has_key(row[1]):
                if tracked_stat_map[row[1]].has_key(row[2]):
                    tracked_stat_map[row[1]][row[2]].append(tracked_row)
                else:
                    tracked_stat_map[row[1]][row[2]]=[]
                    tracked_stat_map[row[1]][row[2]].append(tracked_row)
            else:
                tracked_stat_map[row[1]] = {}
                tracked_stat_map[row[1]][row[2]]=[]
                tracked_stat_map[row[1]][row[2]].append(tracked_row)



        data_list=[]
        for key in tracked_stat_map:
            row=[]
            for tracked_key in tracked_people_list:
                if tracked_stat_map[key].has_key(tracked_key):
                    score_avg=0
                    face_score_avg = 0
                    shirt_color_score_avg = 0
                    trouser_color_score_avg = 0
                    pose_score_avg = 0
                    for record in tracked_stat_map[key][tracked_key]:
                        score_avg=score_avg+record[1]
                        face_score_avg=face_score_avg+record[2]
                        shirt_color_score_avg=shirt_color_score_avg+record[3]
                        trouser_color_score_avg=trouser_color_score_avg+record[4]
                        pose_score_avg=pose_score_avg+record[5]
                    nb_elt=len(tracked_stat_map[key][tracked_key])
                    row.append(score_avg/float(nb_elt))
                    row.append(face_score_avg / float(nb_elt))
                    row.append( shirt_color_score_avg / float(nb_elt))
                    row.append(trouser_color_score_avg / float(nb_elt))
                    row.append( pose_score_avg / float(nb_elt))

                else:
                    row.append(0)
                    row.append(0)
                    row.append(0)
                    row.append(0)
                    row.append(0)
                    #row.append(0)

            data_list.append(row)
        data_np = np.asarray(data_list)




Overleaf        fig, axs = plt.subplots(1, 1)
        #axs[0].set_aspect('equal')
        plt.matshow(data_list, cmap=plt.cm.inferno)
        plt.colorbar()

        # fig, axs = plt.subplots(5, 1)
        # #plt.subplot(311)
        # rospy.loginfo(str(data_matrix))
        # for key in tracked_stat_map:
        #     for key2 in tracked_stat_map[key]:
        #         #score
        #         data_np=np.asarray(tracked_stat_map[key][key2])
        #         axs[0].set_title('General Score')
        #         p=axs[0].plot(data_np[:,0], data_np[:,1], label=key)
        #         #axs[0].fill_between(data_np[:,0],0, data_np[:,2],where= data_np[:,2] >0, alpha=0.5)
        #         tmp = []
        #         tmp2 = []
        #         for row in tracked_stat_map[key][key2]:
        #             tmp.append(float(row[0]))
        #             tmp2.append(float(row[1]))
        #         axs[0].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())
        #         ## Face
        #         data_np = np.asarray(tracked_stat_map[key][key2])
        #         axs[1].set_title('Face Score')
        #         p = axs[1].plot(data_np[:, 0], data_np[:, 2], label=key)
        #         tmp = []
        #         tmp2 = []
        #         for row in tracked_stat_map[key][key2]:
        #             tmp.append(float(row[0]))
        #             tmp2.append(float(row[2]))
        #         axs[1].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())
        #         ## short color
        #         data_np = np.asarray(tracked_stat_map[key][key2])
        #         axs[2].set_title('Shirt Color Score')
        #         p = axs[2].plot(data_np[:, 0], data_np[:, 3], label=key)
        #         tmp = []
        #         tmp2 = []
        #         for row in tracked_stat_map[key][key2]:
        #             tmp.append(float(row[0]))
        #             tmp2.append(float(row[3]))
        #         axs[2].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())
        #         ## trouser color
        #         data_np = np.asarray(tracked_stat_map[key][key2])
        #         axs[3].set_title('Trouser Color Score')
        #         p = axs[3].plot(data_np[:, 0], data_np[:, 4], label=key)
        #         tmp = []
        #         tmp2 = []
        #         for row in tracked_stat_map[key][key2]:
        #             tmp.append(float(row[0]))
        #             tmp2.append(float(row[4]))
        #         axs[3].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())
        #         ## pose score
        #         data_np = np.asarray(tracked_stat_map[key][key2])
        #         axs[4].set_title('Pose Score')
        #         p = axs[4].plot(data_np[:, 0], data_np[:, 5], label=key)
        #         tmp = []
        #         tmp2 = []
        #         for row in tracked_stat_map[key][key2]:
        #             tmp.append(float(row[0]))
        #             tmp2.append(float(row[5]))
        #         axs[4].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())
        #
        #
        #
        #     break

        plt.legend(loc='best')
        plt.show()



    def load_and_display_people_vs_tracked(self,load_folder, file_name):
        real_id_people={}
        real_id_people[0]=[]
        #start, stop, name
        fab_label="FAB"
        raph_label = "RAPH"
        jack_label = "JACK"
        none_label = "NONE"
        real_id_people[0].append([0,115, fab_label])
        real_id_people[0].append([115,170, raph_label])
        real_id_people[0].append([170,199, fab_label])
        real_id_people[0].append([199, 220, raph_label])
        real_id_people[0].append([199, 9999, raph_label])
        real_id_people[1] = []
        real_id_people[1].append([0,115, none_label])
        real_id_people[1].append([115,170, fab_label])
        real_id_people[1].append([170,199, raph_label])
        real_id_people[1].append([199, 220, fab_label])
        real_id_people[1].append([199, 9999, jack_label])
        real_id_people[2] = []
        real_id_people[2].append([0, 115, none_label])
        real_id_people[2].append([115, 170, none_label])
        real_id_people[2].append([170, 199, none_label])
        real_id_people[2].append([199, 220, none_label])
        real_id_people[2].append([199, 9999, fab_label])

        data_matrix = []
        people_tracked_p_map={}
        if os.path.isfile(load_folder + file_name):
            data_matrix.append(np.load(load_folder + file_name))
        else:
            rospy.logwarn("Unable to load stat file : "+ str(load_folder + file_name))
            return

        tracked_people_list=[]

        people_tracked_p_map[fab_label] = {}
        people_tracked_p_map[raph_label] = {}
        people_tracked_p_map[jack_label] = {}
        people_tracked_p_map[none_label] = {}


        for row in data_matrix[0]:
            tracked_row = []
            # time
            tracked_row.append(float(row[0]))
            # people id
            #tracked_row.append(row[1])
            # tracked id
            tracked_row.append(row[2])
            # score
            tracked_row.append(float(row[3]))

            if row[2] not in tracked_people_list:
                tracked_people_list.append(row[2])
            if row[1] != 'NONE':
                label = self.get_label_according_id(int(row[1]),float(row[0]),real_id_people)
                if people_tracked_p_map[label].has_key(row[2]):
                    people_tracked_p_map[label][row[2]].append(tracked_row)
                else:
                    people_tracked_p_map[label][row[2]]=[]
                    people_tracked_p_map[label][row[2]].append(tracked_row)

            # if people_tracked_p_map.has_key(row[1]):
            #     if people_tracked_p_map[row[1]].has_key(row[2]):
            #         people_tracked_p_map[row[1]][row[2]].append(tracked_row)
            #     else:
            #         people_tracked_p_map[row[1]][row[2]]=[]
            #         people_tracked_p_map[row[1]][row[2]].append(tracked_row)
            # else:
            #     people_tracked_p_map[row[1]] = {}
            #     people_tracked_p_map[row[1]][row[2]]=[]
            #     people_tracked_p_map[row[1]][row[2]].append(tracked_row)

        data_list = []
        for key in sorted(people_tracked_p_map.keys()):
            row = []
            for tracked_key in tracked_people_list:
                if people_tracked_p_map[key].has_key(tracked_key):
                    nb_elt = len(people_tracked_p_map[key][tracked_key])
                    row.append(float(nb_elt))
                else:
                    row.append(0)
                    # row.append(0)
            data_list.append(row)

        fig, axs = plt.subplots(1, 1)
        # axs[0].set_aspect('equal')
        plt.matshow(data_list, cmap=plt.cm.YlGnBu)
        plt.colorbar()
        plt.legend(loc='best')
        plt.show()

    def get_label_according_id(self,id,time_elapsed, label_map):
        #real_id_people[0].append([0,115, fab_label])
        for data in label_map[id]:
            if time_elapsed >= data[0] and time_elapsed <data[1]:
                return data[2]


def main():
    #""" main function
    #"""

    load_folder = '/tmp/'
    file_name='3-PEOPLE_TRACKED_STAT.npy'
    file_name2 = '3-PEOPLE_TRACKED_STAT_VS_TRACKED.npy'
    file_name3= '3-PEOPLE_VS_TRACKED.npy'

    statMng = StatMng(load_folder)
    statMng.load_and_display(load_folder,file_name)
    statMng.load_and_display_vs(load_folder, file_name2)
    statMng.load_and_display_people_vs_tracked(load_folder, file_name3)

if __name__ == '__main__':
    main()