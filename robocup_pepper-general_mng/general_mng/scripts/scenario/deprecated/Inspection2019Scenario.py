__author__ = 'Benoit Renault'
import rospy

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService
from LocalManagerWrapper import LocalManagerWrapper

import json
import time
import math


class Inspection2019Scenario(AbstractScenario, AbstractScenarioBus,
                             AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)
        # self._lm_wrapper = LocalManagerWrapper(config.ip_address, config.tcp_port, config.prefix)

        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("pepper4", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        ws = "/home/xia0ben/pepper_ws"
        # ws = "/home/astro/catkin_robocup2019"
        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/inspection/scenario.json".format(ws)) as data:
            self._scenario = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/locations.json".format(ws)) as data:
            self._locations = json.load(data)

        # Scenario data
        self.steps = None

        # - Constants
        self._living_room = self.find_by_id(self._locations, "livingRoom")
        self._entrance = self.find_by_id(self._locations, "entrance")

        # Debug options
        self.allow_navigation = True

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        self.steps = self._scenario["steps"]

        ##################################################################################################
        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
            self.steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        # scenario_start_time = time.time()

        ###################################################################################################

        # Wait for door to open
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["waitDoorOpen"], self.NO_TIMEOUT)

        # - Detect door opening
        waitdooropen_detect = self.find_by_id(self.steps, "waitdooropen_detect")
        self.waitForDoorToOpen(float(waitdooropen_detect["arguments"]["check_freq"]))

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["waitDoorOpen"], self.NO_TIMEOUT)

        ###################################################################################################

        # Go to inspection point
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["goToInspection"], self.NO_TIMEOUT)

        # - Go to inspection point
        gotoinspection_go_to = self.find_by_id(self.steps, "gotoinspection_go_to")
        self._lm_wrapper.go_to(gotoinspection_go_to["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", gotoinspection_go_to["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["goToInspection"], self.NO_TIMEOUT)

        ###################################################################################################

        # Wait for inspection
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["waitInspection"], self.NO_TIMEOUT)

        # - Wait for validation of inspection by referee
        waitinspection_validation = self.find_by_id(self.steps, "waitinspection_validation")
        self._lm_wrapper.ask_open_door(waitinspection_validation["speech"], self.NO_TIMEOUT)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["waitInspection"], self.NO_TIMEOUT)

        ###################################################################################################

        # - Go to Entrance
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["goToEntrance"], self.NO_TIMEOUT)

        # - Go to entrance
        gotoentrance_go_to = self.find_by_id(self.steps, "gotoinspection_go_to")
        self._lm_wrapper.go_to(gotoentrance_go_to["speech"], self._entrance, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", gotoentrance_go_to["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["goToEntrance"], self.NO_TIMEOUT)

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
        self._enableObjectDetectionMngAction = False
        self._enableLookAtObjectMngAction = False
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = False
        self._enableGetPeopleNameAction = False

        self._enableMoveHeadPoseService = True
        self._enableMoveTurnService = False
        self._enablePointAtService = False
        self._enableResetPersonMetaInfoMap = False
        self._enableMinFrontValueService = True

        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)

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
