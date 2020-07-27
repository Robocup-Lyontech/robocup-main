#!/usr/bin/env python
__author__ = 'Benoit Renault & Thomas CURE'
import rospy
import collections

from AbstractScenario import AbstractScenario
from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception
from meta_lib.LTSimulation import LTSimulation
from meta_behaviour.LTHighBehaviour import LTHighBehaviour


import sys
import json
import time
import math
from copy import deepcopy
from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalStatus
import os
from socketIO_client import SocketIO, LoggingNamespace


def singleton(cls): 
    """
    Enables the system to create at most one instance of the class. Two instances of the same class can't be running at the same time.
    """   
    instance = [None]
    def wrapper(*args, **kwargs):
        if instance[0] is None:
            instance[0] = cls(*args, **kwargs)
        return instance[0]

    return wrapper

@singleton
class Receptionist2020CPEScenario(AbstractScenario):

    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config, scenario_path_folder):
        """
        Initializes the scenario Receptionist and receives the needed parameters to run the scenario.
        :param config: contains all the data of the scenario : steps list, parameters etc ...
        :type config: dict
        :param scenario_path_folder: path where is stored the JSON scenario file
        :type scenario_path_folder: string
        """
        self.initScenario()

        self._scenario_path_folder = scenario_path_folder
        self._scenario=config

        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._lm_wrapper = LTHriManagerPalbator(_name_action_server_HRI)

        
        self._drinks = self._scenario['imports']['drinks']
        self._locations = self._scenario['imports']['locations']
        self._people = self._scenario['imports']['people']
        self._videos = self._scenario['imports']['videos']

        self._nav_strategy = self._scenario['parameters']['nav_strategy_parameters']
        
        self.debug_variables = self._scenario['parameters']['debug_variables']

        self.path_folder_to_save_imgs = self._scenario['parameters']['path_folder_to_save_imgs']

        self.current_dir_path = os.path.dirname(os.path.realpath(__file__))

        ##### TEST FOR RESTART #####

        # self.sub_restart = rospy.Subscriber("/test_HRI_restart",Bool,self.handle_restart_hri)

        
        ######### FOR DEBUG #############

        self.socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)



        # DEFAULT -> TRUE. TO MDDIFY, SEE JSON SCENARIO FILE
        self.allow_navigation = self.debug_variables['allow_navigation']
        self.allow_perception = self.debug_variables['allow_perception']
        self.allow_simulation = self.debug_variables['allow_simulation']

        if self.allow_navigation and self.allow_perception:
            self.allow_high_behaviour = True
        else:
            self.allow_high_behaviour = False

        #################################

        if self.allow_navigation:
            self._lt_navigation = LTNavigation()
        
        if self.allow_perception:
            self._lt_perception = LTPerception()
            response = self._lt_perception.reset_people_meta_info_map()


        if self.allow_simulation:
            self._lt_simulation = LTSimulation()
            rospy.loginfo("{class_name}: SETTING UP GUESTS FOR SIMULATED SCENARIO".format(class_name=self.__class__.__name__))
            self._lt_simulation.reset_guests_for_receptionist()
            self._lt_simulation.guest_spawner_for_receptionist("G1_entrance")

        if self.allow_high_behaviour:
            if self.allow_simulation:
                self._lt_high_behaviour = LTHighBehaviour(execution_mode="simulation",socket=self.socketIO)
            else:
                self._lt_high_behaviour = LTHighBehaviour(execution_mode="real",socket=self.socketIO)

        
        rospy.loginfo("{class_name}: JSON FILES LOADED.".format(class_name=self.__class__.__name__))
        # Scenario data
        self.people_name_by_id = {}
        self.people_drink_by_id = {}
        self.people_age_by_id = {}
        self.people_face_path_by_id = {}
        self.steps = None
        self.exit_scenario=False
        self.current_guest=None

        # - Constants
        # self._living_room = self.find_by_id(self._locations, "livingRoom")
        # self._entrance = self.find_by_id(self._locations, "entrance")
        self._path_guests_infos = self._scenario['variables']['guestsInfos']

        known_person = self._scenario['variables']['known_person']
        rospy.loginfo("{class_name} : LEARNING KNOWN PEOPLE FACES".format(class_name=self.__class__.__name__))
        for key in known_person.keys():
            self.people_name_by_id[key] = known_person[key]['name']
            self.people_drink_by_id[key] = known_person[key]['drink']
            self.people_age_by_id[key] = known_person[key]['age']

            rospy.loginfo("{class_name} : LEARNING %s FACE".format(class_name=self.__class__.__name__),str(known_person[key]['name']))
            img_path = os.path.join(self.current_dir_path,self.path_folder_to_save_imgs)
            if self.allow_simulation:
                img_path = os.path.join(img_path,"img/people/"+key+"_simu.png")
            else:
                img_path = os.path.join(img_path,"img/people/"+key+".png")

            if self.allow_perception:    
                response = self._lt_perception.learn_people_meta_from_img_path(img_path,known_person[key]['name'],10)

        rospy.loginfo("{class_name} : SCN : DATA STORED: person: ".format(class_name=self.__class__.__name__)+str(self.people_name_by_id) +" drink : "+str(self.people_drink_by_id)+" age : "+str(self.people_age_by_id))

        rospy.loginfo("{class_name} : CONSTANTS SET.".format(class_name=self.__class__.__name__))

        # - Variables
        # self.people_name_by_id[1] = "Placeholder name"
        # self.people_name_by_id[2] = "Placeholder name"
        # self.people_age_by_id[1] = 1000
        # self.people_age_by_id[2] = 1000
        # self.people_drink_by_id[1] = "Placeholder drink"
        # self.people_drink_by_id[2] = "Placeholder drink"

        # Debug options

        self.configuration_ready = True

    # def handle_restart_hri(self,req):
    #     if self.allow_navigation:
    #         try:
    #             rospy.logwarn("CANCELLING NAV GOALS")
    #             self._lt_navigation.cancel_current_goals()
    #         except Exception as e:
    #             rospy.logerr("Error : %s",e)



    def gm_main_menu(self,indexStep):
        rospy.loginfo("{class_name} : SCN ACTION FOUND MAIN MENU".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)
        rospy.sleep(3)
        result={
                "NextIndex": indexStep+1
        }
        return result

    def gm_found_guest(self,indexStep):
        """
        Function dealing with the foundGuest action. The robot loads a view with the picture of the new guest he just took.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION FOUND NEW GUEST".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self.people_face_path_by_id),self.NO_TIMEOUT)
        rospy.sleep(1)
        result={
                "NextIndex": indexStep+2
        }
        return result

    def gm_found_anyone(self,indexStep):
        """
        Function dealing with the foundAnyone action. The robot just loads the view saying it didn't find anyone and ends the scenario.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION NOT FOUND NEW GUEST".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)
        rospy.sleep(1)
        result={
                "NextIndex": self.end_step_index
        }
        return result


    def gm_look_for_known_guest(self,indexStep):
        """
        Function dealing with the lookForKnownGuest action. The robot will look for a known guest.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION LOOK FOR KNOWN GUEST".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._guest_infos),self.NO_TIMEOUT)
        
        
        guest_to_find = self.steps[indexStep]['arguments']['key']

        people_name = self._guest_infos[guest_to_find]['name']
        if self.allow_high_behaviour:
            response = self._lt_high_behaviour.turn_around_and_detect_someone(people_name)
        elif self.allow_perception:
            response_perception = self._lt_perception.detect_meta_people_from_img_topic(timeout=20)

            rospy.logwarn("RESPONSE PEOPLE : %s",str(response_perception.payload))

            #### FOR DEBUG : GET LABEL AND SCORE TO PRINT IT ON TABLET####
            json_data = {
                "people_list": []
            }
            peopleList = response_perception.payload.peopleMetaList.peopleList
            if peopleList:
                for people in peopleList:
                    json_data['people_list'].append({
                        "name": people.label_id,
                        "score": people.label_score
                    })


            self.socketIO.emit("sendPeopleListDebug",json_data,broadcast=True)
            ###########
            

        rospy.sleep(1)
        result={
                "NextIndex": indexStep+1
        }
        return result


    def gm_look_for_guest(self,indexStep):
        """
        Function dealing with the lookForGuest action. The robot will look for a new guest to ask him infos.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION LOOK FOR GUEST".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)

        guest_to_find = self.steps[indexStep]['arguments']['who']
        
        detection_result = None
        count = 0
        if self.allow_perception:
            while (detection_result is None or detection_result == {}) and count < 5:
                response = self._lt_perception.detect_meta_people_from_img_topic(timeout=20)
            
                detection_result = response.payload

                if not detection_result is None:
                    if detection_result == {}:
                        rospy.logerr("{class_name} : EMPTY RESULT. RETRY".format(class_name=self.__class__.__name__))

                    else:
                        rospy.logwarn("{class_name} : PEOPLE LIST DETECTION %s".format(class_name=self.__class__.__name__),str(response.payload.peopleMetaList.peopleList))
                        # detection = response.payload.peopleMetaList.peopleList[0]

                        # if detection.label_id == "Unknown":
                        
                        #### FOR DEBUG : GET LABEL AND SCORE TO PRINT IT ON TABLET####
                        json_data = {
                            "people_list": []
                        }
                        peopleList = response.payload.peopleMetaList.peopleList
                        if peopleList:
                            for people in peopleList:
                                json_data['people_list'].append({
                                    "name": people.label_id,
                                    "score": people.label_score
                                })

                        self.socketIO.emit("sendPeopleListDebug",json_data,broadcast=True)
                        ###########
                            

                        img_path = os.path.join(self.current_dir_path,self.path_folder_to_save_imgs)
                        img_path = os.path.join(img_path,"img/people/"+guest_to_find+".png")
                        self._lt_perception.take_picture_and_save_it_Palbator(img_path)
                        result={
                            "NextIndex": indexStep+1
                        }
                        self.people_face_path_by_id[guest_to_find] = "img/people/"+guest_to_find+".png"
                        break
                        # else:
                        #     rospy.logwarn("{class_name} : I DETECTED %s BUT I ALREADY KNOW THIS PERSON".format(class_name=self.__class__.__name__),str(detection.label_id))
                        #     result={
                        #         "NextIndex": indexStep+2
                        #     }
                        #     break
                count+=1

            if detection_result is None or detection_result == {}:
                    rospy.logerr("{class_name} : NO GUEST DETECTED NONE RESULT".format(class_name=self.__class__.__name__))
                    result={
                        "NextIndex": indexStep+2
                    }
        else:
            result={
                "NextIndex": indexStep+1
            }
        return result

    def gm_wait(self,indexStep):
        """
        Function dealing with the Wait action. The robot just waits for a timer to end.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION WAIT".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)
        time.sleep(1)
        result={
            "NextIndex": indexStep+1
        }
        return result

    def gm_ask_open_door(self,indexStep):
        """
        Function dealing with the askOpenDoor action. The robot waits the referee to open the door and say Next or click on the tablet.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION ASK OPEN DOOR".format(class_name=self.__class__.__name__))
        return self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)[1]

    def gm_ask_to_follow(self,indexStep):
        """
        Function dealing with the askToFollow action. The robot invites the guest to follow him.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION ASK TO FOLLOW".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._guest_infos),self.NO_TIMEOUT)
        time.sleep(1)
        if self.allow_simulation:
            if self.steps[indexStep]['arguments']['key'] == "Guest_1":
                self._lt_simulation.guest_spawner_for_receptionist("G1_before_present")
            elif self.steps[indexStep]['arguments']['key'] == "Guest_2":
                self._lt_simulation.guest_spawner_for_receptionist("G2_before_present")
                
        result={
            "NextIndex": indexStep+1
        }
        return result

    def gm_go_to(self,indexStep):
        """
        Function dealing with the goTo action. The robot goes to the choosen interest point depending on the scenario.

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION GO TO".format(class_name=self.__class__.__name__))
        result = self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._locations),self.NO_TIMEOUT)[1]

        # if result['NextToDo'] == "RESTART":
        #     return result
        # else:
     
        destination = result['destination']
        destination = destination.title()
        itp_name = ''
        for item in self._locations:
            if item['name'] == destination:
                itp_name = item['interestPoint']
                break

        if self.allow_navigation:
            self._lt_navigation.send_nav_order(self._nav_strategy['action'], self._nav_strategy['mode'], itp_name, self._nav_strategy['timeout'])
        else:
            rospy.logwarn("{class_name} : SEND NAV GOAL : ".format(class_name=self.__class__.__name__) + self._nav_strategy['action'] + " " + self._nav_strategy['mode'] + " ITP : "+itp_name+" timeout= " + str(self._nav_strategy['timeout']))
            time.sleep(2)
        
        return result

    def gm_point_to(self,indexStep):
        """
        Function dealing with the pointTo action. The robot points to something (ex: an empty chair) or to someone (ex: a guest)

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION POINT TO".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._guest_infos),self.NO_TIMEOUT)
        time.sleep(1)
        result={
            "NextIndex": indexStep+1
        }
        return result

    def gm_present_person(self,indexStep):
        """
        Function dealing with the presentPerson action. The robot introduces the new guest to the others.

        :param indexStep: Step index
        :type indexStep: int
        """
        # data = deepcopy(self.steps[indexStep]['arguments']['to'][0]['name'])
        # key = data.split("_name")[0]
        # people_name = self._guest_infos[key]['name']
        # if self.allow_high_behaviour:
        #     response = self._lt_high_behaviour.turn_around_and_detect_someone(people_name)
        # if "John" in self.steps[indexStep]['arguments']['to'][0]['name']:
        #     people_name = "John"
        # elif "Guest_1" in self.steps[indexStep]['arguments']['to'][0]['name']:
        #     people_name = self._guest_infos['Guest_1']['name']
        # elif "Guest_2" in self.steps[indexStep]['arguments']['to'][0]['name']:
        #     people_name = self._guest_infos['Guest_2']['name']

        rospy.loginfo("{class_name} : SCN ACTION PRESENT PERSON".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._guest_infos),self.NO_TIMEOUT)
        time.sleep(1)
        result={
            "NextIndex": indexStep+1
        }
        return result

    def gm_find(self,indexStep):
        """
        Function dealing with the find action. The robot finds something (ex: an empty chair) or someone (ex: a guest)

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION FIND".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step(indexStep,self.NO_TIMEOUT)
        time.sleep(1)
        result={
            "NextIndex": indexStep+1
        }
        return result

    def gm_seat_guest(self,indexStep):
        """
        Function dealing with the seatGuest action. The robot invites the guest to seat down on the seat he was pointing

        :param indexStep: Step index
        :type indexStep: int
        """
        rospy.loginfo("{class_name} : SCN ACTION SEAT GUEST".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_set_current_step_with_data(indexStep,deepcopy(self._guest_infos),self.NO_TIMEOUT)
        time.sleep(1)
        if "Guest_1" in self.steps[indexStep]['arguments']['who']['name'] and self.allow_simulation:
            self._lt_simulation.guest_spawner_for_receptionist("G1_after_present")
            self._lt_simulation.guest_spawner_for_receptionist("G2_entrance")
        result={
            "NextIndex": indexStep+1
        }
        return result

    def action_parser(self,action):
        """
        Load a function according to the action name of current step. 

        :param action: action name of current step
        :type action: string
        """
        rospy.loginfo('-------------------------')
        rospy.loginfo('{class_name} : using parser'.format(class_name=self.__class__.__name__))
        rospy.loginfo('***********')
        switcher = {
        "wait": self.gm_wait,
        "askOpenDoor": self.gm_ask_open_door,
        "askToFollow": self.gm_ask_to_follow,
        "goTo": self.gm_go_to,
        "pointTo": self.gm_point_to,
        "presentPerson": self.gm_present_person,
        "find": self.gm_find,
        "seatGuest": self.gm_seat_guest,
        "lookForGuest": self.gm_look_for_guest,
        "foundGuest": self.gm_found_guest,
        "foundAnyone": self.gm_found_anyone,
        "lookForKnownGuest": self.gm_look_for_known_guest,
        "mainMenuPalbator": self.gm_main_menu
        }
        # Get the function from switcher dictionary
        func = switcher.get(action, lambda: "Invalid action")
        
        return func(self.current_index_scenario)


    def store_guest_in_JSON(self,data):
        """
        Write down guest data sent by HRI to a JSON file. 

        :param data: guest data sent by HRI
        :type data: dict
        """
        self.people_name_by_id[data['who']]=data['name']
        self.people_drink_by_id[data['who']]=data['drink']
        self.people_age_by_id[data['who']]=data['age']

        rospy.loginfo("{class_name} : LEARNING %s FACE".format(class_name=self.__class__.__name__),str(data['name']))
        img_path = os.path.join(self.current_dir_path,self.path_folder_to_save_imgs)
        img_path = os.path.join(img_path,"img/people/"+data['who']+".png")
        if self.allow_perception:
            response = self._lt_perception.learn_people_meta_from_img_path(img_path,data['name'],10)

        with open(os.path.join(self._scenario_path_folder,self._path_guests_infos),"w+") as f:
            data={}
            for key in self.people_name_by_id.keys():
                pathOnTablet=""
                rospy.loginfo("{class_name} : SCN : DRINKS BY ID ".format(class_name=self.__class__.__name__)+str(self.people_drink_by_id[key]))
                for item in self._drinks:
                    drink_guest=deepcopy(self.people_drink_by_id[key]).lower()
                    if drink_guest != "7up":
                        drink_guest=drink_guest.title()
                    if item['name']==drink_guest:
                        pathOnTablet=item['pathOnTablet']
                        break
                
                data[key]={
                    "name": self.people_name_by_id[key],
                    "guestPhotoPath": "img/people/"+key+".png",
                    "drink": self.people_drink_by_id[key],
                    "pathOnTablet": pathOnTablet,
                    "age": self.people_age_by_id[key]
                }
                
            json.dump(data, f, indent=4)
            f.truncate()



        with open(os.path.join(self._scenario_path_folder,self._path_guests_infos),"r") as f:
            self._guest_infos=json.load(f)

    def start_scenario(self):
        """
        Runs the scenario according to the scenario JSON file.
        """

        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))
        self.restart_order=False
        self.steps = deepcopy(self._scenario["steps"])

        self.end_step_index = self.steps[-1]["order"] - 1

        rospy.logerr("END STEP INDEX : %s",str(self.end_step_index))

        ###################################################################################################
        # Reset people database
        # self.resetPeopleMetaInfoMap()

        ##################################################################################################
        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        # step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
        #     self.steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        rospy.loginfo("{class_name} : SCN : WAITING FOR ACTION SERVER ACTIVATION".format(class_name=self.__class__.__name__))
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        self._lm_wrapper.restart_hri(self.NO_TIMEOUT)


        rospy.loginfo("{class_name} : SCN : LOADING CONFIG FOR SCENARIO".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)


        # self.steps=ordered_steps_list['data']
        # self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        self.current_index_scenario=0
        self.scenario_end = False
        while self.scenario_end == False and not rospy.is_shutdown():
            
            rospy.loginfo("{class_name} : SCN : CURRENT STEP INDEX : ".format(class_name=self.__class__.__name__)+str(self.current_index_scenario))
            rospy.loginfo("{class_name} : NEW STEP".format(class_name=self.__class__.__name__))

            self.current_step=deepcopy(self.steps[self.current_index_scenario])

            if self.current_step['action']!="":
                result=self.action_parser(self.current_step['action'])
                
                rospy.loginfo("{class_name} : SCN : RESULT FROM PARSER ".format(class_name=self.__class__.__name__)+str(result))
            else:
                result=self._lm_wrapper.timeboard_set_current_step(self.current_index_scenario,self.NO_TIMEOUT)[1]
                rospy.loginfo("{class_name} : SCN : RESULT WITHOUT PARSER ".format(class_name=self.__class__.__name__)+str(result))

            
            
            if result is None:
                rospy.logwarn("{class_name} : SCN : ACTION ABORTED".format(class_name=self.__class__.__name__))
                break
            
            elif result != None:
                if 'result' in result and result['result']=="PREEMPTED": 
                    rospy.logwarn("{class_name} : SCN : ACTION ABORTED".format(class_name=self.__class__.__name__))
                    break

                # if 'NextToDo' in result and result['NextToDo'] == "RESTART":
                #     rospy.logwarn("{class_name} : HRI RESTART END OF SCENARIO".format(class_name=self.__class__.__name__))
                #     break
                
                if result['NextIndex']!="":
                    self.current_index_scenario=deepcopy(result['NextIndex'])
                else:
                    self.scenario_end = True

                if 'saveData' in result:
                    self.store_guest_in_JSON(result['saveData'])
            
                

            # self.process_json_result(result)
            # if self.exit_scenario==True:
            #     self.exit_scenario=False
            #     break

                
        # scenario_start_time = time.time() 

        ###################################################################################################

        # - Go to door
        # TODO Add scenario step !
        # self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "GPRS_PEOPLE_ENTRANCE_It0", 50.0)

        ###################################################################################################

        # Find first guest
        # # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FindG1"], self.NO_TIMEOUT)
        
        
        # # self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk

        # global_step_find_g1_start_time = time.time()

        # # - Wait
        # findg1_wait = self.find_by_id(self.steps, "findg1_wait")
        # self._lm_wrapper.wait(findg1_wait["speech"], findg1_wait["arguments"]["time"], findg1_wait["arguments"]["time"] + 2.0)

        # # - Ask referee to open the door
        # findg1_ask_referee = self.find_by_id(self.steps, "findg1_ask-referee-to-open-the-door")
        # self._lm_wrapper.ask_open_door(findg1_ask_referee["speech"], self.NO_TIMEOUT)

        # # - Wait2
        # findg1_wait2 = self.find_by_id(self.steps, "findg1_wait2")
        # self._lm_wrapper.wait(findg1_wait2["speech"], findg1_wait2["arguments"]["time"], findg1_wait2["arguments"]["time"] + 2.0)

        # # - Detect human
        # # findg1_detect_human = self.find_step(self.steps, "findg1_detect-human")
        # # self._lm_wrapper.call_human(findg1_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FindG1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # # Ask infos about first guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # # global_step_ask_info_g1_start_time = time.time()

        # #Head's up
        # self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        # #First Ask name
        # askinfog1_ask_name_max_counts = 3  # TODO Move this as config parameter
        # askinfog1_ask_name = self.find_by_id(self.steps, "askinfog1_ask-name")
        # askinfog1_confirm_name = self.find_by_id(self.steps, "askinfog1_confirm-name")
        # self.people_name_by_id[1] = self.ask_name_and_confirm(
        #     askinfog1_ask_name_max_counts, askinfog1_ask_name, askinfog1_confirm_name)

        # # Learn face from name
        # # TODO whatif the face is not properly seen ? --> Make specific scenario view that sends feedback !
        # state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.people_name_by_id[1], 10.0)

        # # Then ask drink
        # askinfog1_ask_drink_max_counts = 3  # TODO Move this as config parameter
        # askinfog1_ask_drink = self.find_by_id(self.steps, "askinfog1_ask-drink")
        # askinfog1_confirm_drink = self.find_by_id(self.steps, "askinfog1_confirm-drink")
        # self.people_drink_by_id[1] = self.ask_drink_and_confirm(askinfog1_ask_drink_max_counts, askinfog1_ask_drink, askinfog1_confirm_drink, self.people_name_by_id[1])

        # # - Ask age
        # askinfog1_ask_age = self.find_by_id(self.steps, "askinfog1_ask-age")
        # askinfog1_ask_age_speech = askinfog1_ask_age["speech"]
        # askinfog1_ask_age_speech["name"] = self.people_name_by_id[1]
        # self.people_age_by_id[1] = self._lm_wrapper.ask_age(askinfog1_ask_age_speech, self.NO_TIMEOUT)[1]

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # # Go to living room
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        # # global_step_go_to_lr1_start_time = time.time()

        # # - Ask to follow
        # gotolr1_ask_to_follow = self.find_by_id(self.steps, "gotolr1_ask-to-follow")
        # gotolr1_ask_to_follow["speech"]["name"] = self.people_name_by_id[1]
        # self._lm_wrapper.ask_to_follow(gotolr1_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # # - Go to living room
        # gotolr1_go_to_living_room = self.find_by_id(self.steps, "gotolr1_go-to-living-room")
        # self._lm_wrapper.go_to(gotolr1_go_to_living_room["speech"], self._living_room, self.NO_TIMEOUT)
        # self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", gotolr1_go_to_living_room["arguments"]["interestPoint"], 50.0)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # # Introduce people
        # self.remap_topic("/pepper_robot/camera/front/image_raw", "/darknet_ros/camera_in", freq=2.0)
        # self.remap_topic("/pepper_robot/camera/front/image_raw", "/openpose/camera_in", freq=2.0)
        # self.introduce_people_to_each_others()

        # ###################################################################################################

        # # Seat first guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG1"], self.NO_TIMEOUT)
        # # global_step_seat_g1_start_time = time.time()

        # # - Find empty seat
        # self.find_an_empty_chair(1)
        # self.unremap_topic("/pepper_robot/camera/front/image_raw", "/darknet_ros/camera_in")
        # self.unremap_topic("/pepper_robot/camera/front/image_raw", "/openpose/camera_in")

        # # - Tell first guest to seat
        # seat_g1 = self.find_by_id(self.steps, "seatg1_tell-first-guest-to-seat")
        # seat_g1["speech"]["name"] = self.people_name_by_id[1]
        # self._lm_wrapper.seat_guest(seat_g1["speech"], self.NO_TIMEOUT)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # # TODO If time too short, don't try to bring second guest ?

        # # Go to door
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)
        # # global_step_find_go_to_door1_start_time = time.time()

        # # - Go to door
        # gotodoor1 = self.find_by_id(self.steps, "gotodoor1_go-to-door")
        # self._lm_wrapper.go_to(gotodoor1["speech"], self._entrance, self.NO_TIMEOUT)
        # self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", gotodoor1["arguments"]["interestPoint"], 50.0)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)

        # # ###################################################################################################

        # # Find second guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FindG2"], self.NO_TIMEOUT)
        # self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        # # global_step_find_g2_start_time = time.time()

        # # - Wait
        # findg2_wait = self.find_by_id(self.steps, "findg2_wait")
        # self._lm_wrapper.wait(findg2_wait["speech"], findg2_wait["arguments"]["time"], findg2_wait["arguments"]["time"] + 2.0)

        # # - Ask referee to open the door
        # findg2_ask_referee = self.find_by_id(self.steps, "findg2_ask-referee-to-open-the-door")
        # self._lm_wrapper.ask_open_door(findg2_ask_referee["speech"], self.NO_TIMEOUT)

        # # - Wait2
        # findg2_wait2 = self.find_by_id(self.steps, "findg2_wait2")
        # self._lm_wrapper.wait(findg2_wait2["speech"], findg2_wait2["arguments"]["time"], findg2_wait2["arguments"]["time"] + 2.0)

        # # - Detect human
        # # findg2_detect_human = self.find_step(self.steps, "findg2_detect-human")
        # # self._lm_wrapper.call_human(findg2_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FindG2"], self.NO_TIMEOUT)

        # # ###################################################################################################

        # # Ask infos about second guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)
        # # global_step_ask_info_g2_start_time = time.time()

        # #Head's up
        # self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        # #First Ask name
        # askinfog2_ask_name_max_counts = 3  # TODO Move this as config parameter
        # askinfog2_ask_name = self.find_by_id(self.steps, "askinfog2_ask-name")
        # askinfog2_confirm_name = self.find_by_id(self.steps, "askinfog2_confirm-name")
        # self.people_name_by_id[2] = self.ask_name_and_confirm(
        #     askinfog2_ask_name_max_counts, askinfog2_ask_name, askinfog2_confirm_name)

        # # Learn face from name
        # # TODO whatif the face is not properly seen ? --> Make specific scenario view that sends feedback !
        # state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.people_name_by_id[2], 10.0)

        # # Then ask drink
        # askinfog2_ask_drink_max_counts = 3  # TODO Move this as config parameter
        # askinfog2_ask_drink = self.find_by_id(self.steps, "askinfog2_ask-drink")
        # askinfog2_confirm_drink = self.find_by_id(self.steps, "askinfog2_confirm-drink")
        # self.people_drink_by_id[2] = self.ask_drink_and_confirm(askinfog2_ask_drink_max_counts, askinfog2_ask_drink, askinfog2_confirm_drink, self.people_name_by_id[2])

        # # - Ask age
        # askinfog2_ask_age = self.find_by_id(self.steps, "askinfog2_ask-age")
        # askinfog2_ask_age_speech = askinfog2_ask_age["speech"]
        # askinfog2_ask_age_speech["name"] = self.people_name_by_id[2]
        # self.people_age_by_id[2] = self._lm_wrapper.ask_age(askinfog2_ask_age_speech, self.NO_TIMEOUT)[1]

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)

        # # ###################################################################################################

        # # Go to living room
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)
        # global_step_go_to_lr2_start_time = time.time()

        # # - Ask to follow
        # gotolr2_ask_to_follow = self.find_by_id(self.steps, "gotolr2_ask-to-follow")
        # gotolr2_ask_to_follow["speech"]["name"] = self.people_name_by_id[2]
        # self._lm_wrapper.ask_to_follow(gotolr2_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # # - Go to living room
        # gotolr2_go_to_living_room = self.find_by_id(self.steps, "gotolr2_go-to-living-room")
        # self._lm_wrapper.go_to(gotolr2_go_to_living_room["speech"], self._living_room, self.NO_TIMEOUT)
        # self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", gotolr2_go_to_living_room["arguments"]["interestPoint"], 50.0)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # # Introduce people
        # self.introduce_people_to_each_others()

        # # Seat second guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG2"], self.NO_TIMEOUT)
        # global_step_seat_g2_start_time = time.time()

        # # - Find empty seat
        # self.find_an_empty_chair(2)

        # # - Tell first guest to seat
        # seat_g2 = self.find_by_id(self.steps, "seatg2_tell-first-guest-to-seat")
        # seat_g2["speech"]["name"] = self.people_name_by_id[2]
        # self._lm_wrapper.seat_guest(seat_g2["speech"], self.NO_TIMEOUT)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG2"], self.NO_TIMEOUT)

        # # ###################################################################################################

        # # Finish Scenario
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

        # # - Finish Scenario
        # # self.send_action_to_local_manager("finishscenario_finish-scenario")

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

        rospy.loginfo("""
                ######################################
                Finished executing the {scenario_name} Scenario...
                ######################################
                """.format(scenario_name=self._scenario["name"]))
        rospy.loginfo("SCN : DATA STORED: person: "+str(self.people_name_by_id) +" drink : "+str(self.people_drink_by_id)+" age : "+str(self.people_age_by_id))
    # def gmBusListener(self, msg):
    #     if self._status == self.WAIT_ACTION_STATUS:
    #         self.checkActionStatus(msg)

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

        # AbstractScenarioAction.configure_intern(self)
        # AbstractScenarioService.configure_intern(self)

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

    # def ask_name_and_confirm(self, ask_name_max_count, ask_step_data, confirm_step_data):
    #     ask_name_counter = 0
    #     while True:

    #         # - Ask name
    #         tentative_guest_name = self._lm_wrapper.ask_name(ask_step_data["speech"], self._guest_infos, self.NO_TIMEOUT)[1]

    #         # - Confirm name
    #         confirm_speech = confirm_step_data["speech"]
    #         confirm_speech["name"] = tentative_guest_name
    #         ask_name_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
    #         if ask_name_confirmed:
    #             rospy.loginfo("Guest got name {name} confirmed !".format(name=tentative_guest_name))
    #             return tentative_guest_name

    #         ask_name_counter += 1

    #         if ask_name_counter >= ask_name_max_count:
    #             rospy.logwarn("Could not get name with confirmation !")
    #             # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
    #             #  I guess I will just call you {Last_Understood Name}
    #             return tentative_guest_name

    # def ask_drink_and_confirm(self, ask_drink_max_count, ask_step_data, confirm_step_data, guest_name):
    #     ask_drink_counter = 0
    #     while True:
    #         # - Ask drink
    #         ask_speech = ask_step_data["speech"]
    #         ask_speech["name"] = guest_name
    #         tentative_guest_drink = self._lm_wrapper.ask_drink(ask_speech, self._drinks, self.NO_TIMEOUT)[1]

    #         # - Confirm drink
    #         confirm_speech = confirm_step_data["speech"]
    #         confirm_speech["drink"] = tentative_guest_drink["name"]
    #         ask_drink_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
    #         if ask_drink_confirmed:
    #             rospy.loginfo("Guest got drink <{drink}> confirmed !".format(drink=tentative_guest_drink["name"]))
    #             return tentative_guest_drink

    #         ask_drink_counter += 1

    #         if ask_drink_counter >= ask_drink_max_count:
    #             rospy.logwarn("Could not get drink with confirmation !")
    #             # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
    #             #  I guess I will just consider you like {Last_Understood Drink}
    #             return tentative_guest_drink

    # def introduce_people_to_each_others(self):
    #     """
    #     Pepper turn on himself to find people and to introduce the to the group
    #     """
    #     # Intialize guests / host presentation
    #     nb_people_here = len(self.people_name_by_id.keys())
    #     nb_people_introduced = 0
    #     people_introduced = {}
    #     for name in self.people_name_by_id.values():
    #         people_introduced[name] = False
    #     newbie_name = self.people_name_by_id[max(self.people_name_by_id.keys())]
    #     # Set head position
    #     self.moveheadPose(self.HEAD_PITCH_CENTER, self.HEAD_YAW_CENTER, True)
    #     # Turn around to introduce guests
    #     # angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, 200.0, -225.0, 250.0, -275.0, 300.0, -325.0, 350.0]
    #     angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0]
    #     for angle in angle_list:
    #         # Find people in the image
    #         state_getObject, result_getObject = self.getObjectInFrontRobot(["person"], False, 50.0)
    #         # Loop on people found
    #         if result_getObject is not None:
    #             if len(result_getObject.labelFound) > 0:
    #                 # TODO Ajouter un move head pour viser le visage
    #                 # Get people names
    #                 state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgTopic(50.0)
    #                 # If we recognize a face
    #                 if result_getPeopleName is not None:
    #                     if len(result_getPeopleName.peopleNames) > 0:
    #                         # Test
    #                         print result_getPeopleName.peopleNames
    #                         print result_getPeopleName.peopleNamesScore
    #                         # Compute people bounding box area
    #                         name_by_area = {}
    #                         for people, name in zip(result_getPeopleName.peopleMetaList.peopleList, result_getPeopleName.peopleNames):
    #                             box_x0 = people.details.boundingBox.points[0].x
    #                             box_x1 = people.details.boundingBox.points[1].x
    #                             box_y0 = people.details.boundingBox.points[0].y
    #                             box_y1 = people.details.boundingBox.points[1].y
    #                             box_area = abs(box_x0 - box_x1)*abs(box_y0 - box_y1)
    #                             name_by_area[box_area] = name
    #                         # Inverse sort of the people aera : closest people first
    #                         name_by_area_ordered = collections.OrderedDict(sorted(name_by_area.items(), reverse=True))
    #                         # Test all the names we found
    #                         for (name_of_people_found, i_name_of_people_found) in zip(name_by_area_ordered.values(), range(len(name_by_area_ordered.values()))):
    #                             # If the persone has not been recognize we jump to the next one
    #                             if name_of_people_found == "None":
    #                                 continue
    #                             # Checked if we find correspondences with any known name
    #                             for name_of_people_known in self.people_name_by_id.values():
    #                                 # TODO
    #                                 # First possibility : The person found is the new guest to introduce to everyone
    #                                 if (   (name_of_people_found == name_of_people_known)
    #                                    and (people_introduced[name_of_people_known] == False)
    #                                    and (name_of_people_found == newbie_name)):
    #                                     # Point to Guest
    #                                     state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
    #                                     # Introduce new_guest_to_john
    #                                     guest_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(name_of_people_known)]
    #                                     self.introduce_guest_to_host(guest_id)
    #                                     # Release arm
    #                                     self.releaseArms()
    #                                     # Update internal variables
    #                                     nb_people_introduced += 1
    #                                     people_introduced[name_of_people_known] = True
    #                                 # Second possibility
    #                                 elif (   (name_of_people_found == name_of_people_known)
    #                                      and (people_introduced[name_of_people_known] == False)):
    #                                     # Point to Guest
    #                                     state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
    #                                     # Introduce guests
    #                                     guest1_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(name_of_people_known)]
    #                                     guest2_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(newbie_name)]
    #                                     self.introduce_one_guest_to_another_guest(guest1_id, guest2_id)
    #                                     # Release arm
    #                                     self.releaseArms()
    #                                     # Update internal variables
    #                                     nb_people_introduced += 1
    #                                     people_introduced[name_of_people_known] = True
    #                                 # Third possibility : The person found is the host
    #                                 # TODO We find the host by elimination : maybe to improve upon
    #                                 elif (   (name_of_people_found == "Unknown")
    #                                      and (people_introduced["John"] == False)):
    #                                     # Point to John
    #                                     state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_name_of_people_found, False, False, 2, 50.0)
    #                                     # Introduce John to new guest
    #                                     guest_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(newbie_name)]
    #                                     self.introduce_host_to_guest(guest_id)
    #                                     # Release arm
    #                                     self.releaseArms()
    #                                     # Update internal variables
    #                                     nb_people_introduced += 1
    #                                     people_introduced["John"] = True
    #                                 else:
    #                                     # Mismatch
    #                                     pass
    #         # Check if everyone has been introduced
    #         if nb_people_introduced < nb_people_here:
    #             # Turn a bit to find someone else
    #             self.moveTurn(angle*math.pi/180.0)
    #             #print "I TURN !!!!"
    #         else:
    #             # End introducing
    #             break
    #     return

    # def introduce_new_guest_to_others(self, guest_id):
    #     """
    #     Introduce the new guest to other guests
    #     """
    #     # TODO: nouveau dialogue
    #     print "INTRODUCING {0} TO OTHERS".format(self.people_name_by_id[guest_id])
    #     pass

    # def introduce_one_guest_to_another_guest(self, guest1_id, guest2_id):
    #     """
    #     Introduce one guest to another guest
    #     """
    #     # TODO: nouveau dialogue
    #     # # Introduce guest to John
    #     # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceG{0}ToG{1}".format(guest1_id, guest2_id)), self.NO_TIMEOUT)
    #     # # Say name and drink
    #     # int_guest_host = self.find_by_id(self.steps, "introduceg{0}tog{1}_say-name-and-drink".format(guest1_id, guest2_id))
    #     # self._lm_wrapper.present_person(int_guest_host["speech"], self.people_name_by_id[guest1_id], self.people_drink_by_id[guest1_id],
    #     #                                 [self.people_name_by_id[guest2_id]], self.NO_TIMEOUT)
    #     # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceG{0}ToG{1}".format(guest_id, guest2_id)), self.NO_TIMEOUT)
    #     print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[guest1_id], self.people_name_by_id[guest1_id])
    #     pass

    # def introduce_host_to_guest(self, guest_id):
    #     """
    #     Introduce the host to a given guest
    #     """
    #     # TODO: Nouveau dialogue
    #     # # Introduce John to first guest
    #     # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceJohnToG{0}".format(guest_id)), self.NO_TIMEOUT)
    #     # # Say name and drink
    #     # int_host_guest = self.find_by_id(self.steps, "introducejohntog{0}_say-name-and-drink".format(guest_id))
    #     # self._lm_wrapper.present_person(int_host_guest["speech"], self.people_name_by_id[0], self.people_drink_by_id[0],
    #     #                                 [self.people_name_by_id[guest_id]], self.NO_TIMEOUT)
    #     # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceJohnToG{0}".format(guest_id)), self.NO_TIMEOUT)
    #     print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[0], self.people_name_by_id[guest_id])
    #     pass

    # def introduce_guest_to_host(self, guest_id):
    #     """
    #     Introduce a guest to the host
    #     """
    #     # TODO: nouveau dialogue
    #     # # Introduce guest to John
    #     # self._lm_wrapper.timeboard_set_current_step(self.find_by_id(self.steps, "IntroduceG{0}ToJohn".format(guest_id)), self.NO_TIMEOUT)
    #     # # Say name and drink
    #     # int_guest_host = self.find_by_id(self.steps, "introduceg{0}tojohn_say-name-and-drink".format(guest_id))
    #     # self._lm_wrapper.present_person(int_guest_host["speech"], self.people_name_by_id[guest_id], self.people_drink_by_id[guest_id],
    #     #                                 [self.people_name_by_id[0]], self.NO_TIMEOUT)
    #     # self._lm_wrapper.timeboard_send_step_done(self.find_by_id(self.steps, "IntroduceG{0}ToJohn".format(guest_id)), self.NO_TIMEOUT)
    #     print "INTRODUCING {0} TO {1}".format(self.people_name_by_id[guest_id], self.people_name_by_id[0])
    #     pass

    # # def find_an_empty_chair(self, guest_id):
    # #     """
    # #     Find an empty chair for a guest
    # #     """
    # #     # Set head position
    # #     self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_FOR_CHAIR, self.HEAD_YAW_CENTER, True)
    # #     # Turn around to introduce guests
    # #     # angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, 200.0, -225.0, 250.0, -275.0, 300.0, -325.0, 350.0]
    # #     angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0]
    # #     for angle in angle_list:
    # #         # Find and point the chair in the image
    # #         state_lookAtObject, result_lookAtObject = self.lookAtObject(["chair"], 0, False, False, 2, 50.0)
    # #         # Loop on people found
    # #         if result_lookAtObject is not None:
    # #             if result_lookAtObject.nb_label > 0:
    # #                 self.sit_here_guest(guest_id)
    # #                 self.releaseArms()
    # #                 return
    # #         # Turn a bit to find somewhere else
    # #         self.moveTurn(angle*math.pi/180.0)
    # #     return

    # def sit_here_guest(self, guest_id):
    #     """
    #     Say to a guest to sit here
    #     """
    #     # TODO : To code
    #     print "Sit here {0}".format(self.people_name_by_id[guest_id])
