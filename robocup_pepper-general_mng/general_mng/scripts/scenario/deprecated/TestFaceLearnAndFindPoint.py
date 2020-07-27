__author__ = 'Benoit Renault'
import rospy
import collections

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService
from LocalManagerWrapper import LocalManagerWrapper

import json
import time
import math

import pdb


class TestFaceLearnAndFindPoint(AbstractScenario, AbstractScenarioBus,
                                AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)
        # self._lm_wrapper = LocalManagerWrapper(config.ip_address, config.tcp_port, config.prefix)

        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("pepper2", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        ws = "/home/astro/catkin_robocup2019"
        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json".format(ws)) as data:
            self._scenario = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/drinks.json".format(ws)) as data:
            self._drinks = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/locations.json".format(ws)) as data:
            self._locations = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/people.json".format(ws)) as data:
            self._people = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/videos.json".format(ws)) as data:
            self._videos = json.load(data)

        # Scenario data
        self.people_name_by_id = {}
        self.people_drink_by_id = {}
        self.people_age_by_id = {}
        self.steps = None

        # - Constants
        self._living_room = self.find_by_id(self._locations, "livingRoom")
        self._entrance = self.find_by_id(self._locations, "entrance")
        self.people_name_by_id[0] = "John"
        self.people_age_by_id[0] = 40
        self.people_drink_by_id[0] = self.find_by_id(self._drinks, "coke")

        # - Variables
        # self.people_name_by_id[1] = "Placeholder name"
        # self.people_name_by_id[2] = "Placeholder name"
        # self.people_age_by_id[1] = 1000
        # self.people_age_by_id[2] = 1000
        # self.people_drink_by_id[1] = "Placeholder drink"
        # self.people_drink_by_id[2] = "Placeholder drink"

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        self.steps = self._scenario["steps"]

        ###################################################################################################
        # Reset people database
        self.resetPeopleMetaInfoMap()

        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
            self.steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        # Ask infos about first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # global_step_ask_info_g1_start_time = time.time()

        #Head's up
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        #First Ask name
        askinfog1_ask_name_counter = 0
        askinfog1_ask_name_max_counts = 3
        askinfog1_ask_name = self.find_by_id(self.steps, "askinfog1_ask-name")
        askinfog1_confirm_name = self.find_by_id(self.steps, "askinfog1_confirm-name")
        while True:

            if askinfog1_ask_name_counter >= askinfog1_ask_name_max_counts:
                rospy.logwarn("Could not get name with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just call you {Last_Understood Name}
                break

            # - Ask name
            tentative_guest_1_name = self._lm_wrapper.ask_name(askinfog1_ask_name["speech"], self._people, self.NO_TIMEOUT)[1]

            # - Confirm name
            confirm_speech = askinfog1_confirm_name["speech"]
            confirm_speech["name"] = tentative_guest_1_name
            askinfog1_ask_name_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if askinfog1_ask_name_confirmed:
                rospy.loginfo("Guest 1 got name {name} confirmed !".format(name=tentative_guest_1_name))
                self.people_name_by_id[1] = tentative_guest_1_name
                break

            askinfog1_ask_name_counter += 1

        #Learn face from name
        state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.people_name_by_id[1], 10.0) #TODO whatif the face is not properly seen
        state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.people_name_by_id[1], 10.0) #TODO whatif the face is not properly seen
        print result_learnPeopleMeta

        # Then ask drink
        askinfog1_ask_drink_counter = 0
        askinfog1_ask_drink_max_counts = 3
        askinfog1_ask_drink = self.find_by_id(self.steps, "askinfog1_ask-drink")
        askinfog1_confirm_drink = self.find_by_id(self.steps, "askinfog1_confirm-drink")
        while True:

            if askinfog1_ask_drink_counter >= askinfog1_ask_drink_max_counts:
                rospy.logwarn("Could not get drink with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just consider you like {Last_Understood Drink}
                break

            # - Ask drink
            ask_speech = askinfog1_ask_drink["speech"]
            ask_speech["name"] = self.people_name_by_id[1]
            tentative_guest_1_drink = self._lm_wrapper.ask_drink(ask_speech, self._drinks, self.NO_TIMEOUT)[1]

            # - Confirm drink
            confirm_speech = askinfog1_confirm_drink["speech"]
            confirm_speech["drink"] = tentative_guest_1_drink["name"]
            askinfog1_ask_drink_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if askinfog1_ask_drink_confirmed:
                rospy.loginfo("Guest 1 got drink <{drink}> confirmed !".format(drink=tentative_guest_1_drink["name"]))
                self.people_drink_by_id[1] = tentative_guest_1_drink
                break

            askinfog1_ask_drink_counter += 1

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        ##############################################################################################
        # Point to and introduce both G1 and Host

        # self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk

        self.introduce_people_to_each_others()

        # End

        rospy.loginfo("""
                ######################################
                Finished executing the {scenario_name} Scenario...
                ######################################
                """.format(scenario_name=self._scenario["name"]))

    def gmBusListener(self, msg):
        if self._status == self.WAIT_ACTION_STATUS:
            self.checkActionStatus(msg)

    def initScenario(self):
        self._enableNavAction = True
        self._enableTtsAction = False
        self._enableDialogueAction = False
        self._enableAddInMemoryAction = False
        self._enableObjectDetectionMngAction = True
        self._enableLookAtObjectMngAction = True
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = True
        self._enableGetPeopleNameAction = True

        self._enableMoveHeadPoseService = True
        self._enableMoveTurnService = True
        self._enablePointAtService = True
        self._enableResetPersonMetaInfoMapService = True
        self._enableReleaseArmsService = True

        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)

    def simulate_ros_work(self, time_for_work, log_string):
        rospy.logwarn(log_string)
        rospy.logwarn("Waiting for {duration} seconds...".format(duration=time_for_work))
        time.sleep(time_for_work)

    def find_by_id(self, steps_array, step_id):
        step_index = self.find_index_by_id(steps_array, step_id)
        if step_index is None:
            return None
        else:
            return steps_array[step_index]

    def find_index_by_id(self, steps_array, step_id):
        for index in range(len(steps_array)):
            step = steps_array[index]
            if step["id"] == step_id:
                return index
        return None

    def introduce_people_to_each_others(self):
        """
        Pepper turn on himself to find people and to introduce the to the group
        """
        # Intialize guests / host presentation
        nb_people_here = len(self.people_name_by_id.keys())
        nb_people_introduced = 0
        people_introduced = {}
        for name in self.people_name_by_id.values():
            people_introduced[name] = False
        newbie_name = self.people_name_by_id[max(self.people_name_by_id.keys())]
        # Set head position
        self.moveheadPose(self.HEAD_PITCH_CENTER, self.HEAD_YAW_CENTER, True)
        # Turn around to introduce guests
        # angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, 200.0, -225.0, 250.0, -275.0, 300.0, -325.0, 350.0]
        angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0]
        for angle in angle_list:
            # Find people in the image
            state_getObject, result_getObject = self.getObjectInFrontRobot(["person"], False, 50.0)
            # Loop on people found
            if result_getObject is not None:
                if len(result_getObject.labelFound) > 0:
                    # TODO Ajouter un move head pour viser le visage
                    # Get people names
                    state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgTopic(50.0)
                    # If we recognize a face
                    if result_getPeopleName is not None:
                        if len(result_getPeopleName.peopleNames) > 0:
                            # Test
                            print result_getPeopleName.peopleNames
                            print result_getPeopleName.peopleNamesScore
                            # Compute people bounding box area
                            name_by_area = {}
                            for people, name in zip(result_getPeopleName.peopleMetaList.peopleList, result_getPeopleName.peopleNames):
                                box_x0 = people.details.boundingBox.points[0].x
                                box_x1 = people.details.boundingBox.points[1].x
                                box_y0 = people.details.boundingBox.points[0].y
                                box_y1 = people.details.boundingBox.points[1].y
                                box_area = abs(box_x0 - box_x1)*abs(box_y0 - box_y1)
                                name_by_area[box_area] = name
                            # Inverse sort of the people aera : closest people first
                            name_by_area_ordered = collections.OrderedDict(sorted(name_by_area.items(), reverse=True))
                            # Test all the names we found
                            for (name_of_people_found, i_name_of_people_found) in zip(name_by_area_ordered.values(), range(len(name_by_area_ordered.values()))):
                                # If the persone has not been recognize we jump to the next one
                                if name_of_people_found == "None":
                                    continue
                                # Checked if we find correspondences with any known name
                                for name_of_people_known in self.people_name_by_id.values():
                                    # TODO
                                    # First possibility : The person found is the new guest to introduce to everyone
                                    if (   (name_of_people_found == name_of_people_known)
                                       and (people_introduced[name_of_people_known] == False)
                                       and (name_of_people_found == newbie_name)):
                                        # Point to Guest
                                        state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
                                        # Introduce new_guest_to_john
                                        guest_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(name_of_people_known)]
                                        self.introduce_guest_to_host(guest_id)
                                        # Release arm
                                        self.releaseArms()
                                        # Update internal variables
                                        nb_people_introduced += 1
                                        people_introduced[name_of_people_known] = True
                                    # Second possibility
                                    elif (   (name_of_people_found == name_of_people_known)
                                         and (people_introduced[name_of_people_known] == False)):
                                        # Point to Guest
                                        state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
                                        # Introduce guests
                                        guest1_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(name_of_people_known)]
                                        guest2_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(newbie_name)]
                                        self.introduce_one_guest_to_another_guest(guest1_id, guest2_id)
                                        # Release arm
                                        self.releaseArms()
                                        # Update internal variables
                                        nb_people_introduced += 1
                                        people_introduced[name_of_people_known] = True
                                    # Third possibility : The person found is the host
                                    # TODO We find the host by elimination : maybe to improve upon
                                    elif (   (name_of_people_found == "Unknown")
                                         and (people_introduced["John"] == False)):
                                        # Point to John
                                        state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
                                        # Introduce John to new guest
                                        guest_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(newbie_name)]
                                        self.introduce_host_to_guest(guest_id)
                                        # Release arm
                                        self.releaseArms()
                                        # Update internal variables
                                        nb_people_introduced += 1
                                        people_introduced["John"] = True
                                    else:
                                        # Mismatch
                                        pass
            # Check if everyone has been introduced
            if nb_people_introduced < nb_people_here:
                # Turn a bit to find someone else
                self.moveTurn(angle*math.pi/180.0)
                #print "I TURN !!!!"
            else:
                # End introducing
                break
        return

    def introduce_new_guest_to_others(self, guest_id):
        """
        Introduce the new guest to other guests
        """
        # TODO: nouveau dialogue
        print "INTRODUCING {0} TO OTHERS".format(self.people_name_by_id[guest_id])
        pass

    def introduce_one_guest_to_another_guest(self, guest1_id, guest2_id):
        """
        Introduce one guest to another guest
        """
        # TODO: nouveau dialogue
        # # Introduce guest to John
        # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceG{0}ToG{1}".format(guest1_id, guest2_id)), self.NO_TIMEOUT)
        # # Say name and drink
        # int_guest_host = self.find_by_id(self.steps, "introduceg{0}tog{1}_say-name-and-drink".format(guest1_id, guest2_id))
        # self._lm_wrapper.present_person(int_guest_host["speech"], self.people_name_by_id[guest1_id], self.people_drink_by_id[guest1_id],
        #                                 [self.people_name_by_id[guest2_id]], self.NO_TIMEOUT)
        # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceG{0}ToG{1}".format(guest_id, guest2_id)), self.NO_TIMEOUT)
        print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[guest1_id], self.people_name_by_id[guest1_id])
        pass

    def introduce_host_to_guest(self, guest_id):
        """
        Introduce the host to a given guest
        """
        # TODO: Nouveau dialogue
        # # Introduce John to first guest
        # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceJohnToG{0}".format(guest_id)), self.NO_TIMEOUT)
        # # Say name and drink
        # int_host_guest = self.find_by_id(self.steps, "introducejohntog{0}_say-name-and-drink".format(guest_id))
        # self._lm_wrapper.present_person(int_host_guest["speech"], self.people_name_by_id[0], self.people_drink_by_id[0],
        #                                 [self.people_name_by_id[guest_id]], self.NO_TIMEOUT)
        # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceJohnToG{0}".format(guest_id)), self.NO_TIMEOUT)
        print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[0], self.people_name_by_id[guest_id])
        pass

    def introduce_guest_to_host(self, guest_id):
        """
        Introduce a guest to the host
        """
        # TODO: nouveau dialogue
        # # Introduce guest to John
        # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceG{0}ToJohn".format(guest_id)), self.NO_TIMEOUT)
        # # Say name and drink
        # int_guest_host = self.find_by_id(self.steps, "introduceg{0}tojohn_say-name-and-drink".format(guest_id))
        # self._lm_wrapper.present_person(int_guest_host["speech"], self.people_name_by_id[guest_id], self.people_drink_by_id[guest_id],
        #                                 [self.people_name_by_id[0]], self.NO_TIMEOUT)
        # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceG{0}ToJohn".format(guest_id)), self.NO_TIMEOUT)
        print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[guest_id], self.people_name_by_id[0])
        pass
