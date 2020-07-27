__author__ = 'Benoit Renault'

import json

from AbstractScenario import AbstractScenario

from meta_lib.LTPerception import LTPerception
from meta_lib.LTHriManager import LTHriManager
from meta_lib.LTNavigation import LTNavigation


class DoorOpenAndNavigScenarioV1(AbstractScenario):
    def __init__(self, scenario_data):
        self.configuration_ready = False
        self.lt_hri_manager, self.lt_navigation, self.lt_perception = LTHriManager(), LTNavigation(), LTPerception()

        self._scenario_data = scenario_data
        self._locations = self._scenario_data["imports"]["locations"]
        self._steps = self._scenario_data["steps"]
        self._living_room = self._locations["livingRoom"]
        self._entrance = self._locations["entrance"]

        self.init_scenario(scenario_data)

        if self.lt_perception.configurationReady and self.lt_navigation.configurationReady:
            self.configuration_ready = True

    def init_scenario(self, scenario_data):
        pass

    def start_scenario(self):
        self.print_name(self.__name__)

        # Wait door opening
        result = self.lt_perception.wait_for_door_to_open()
        self.print_result(result)

        # start navigation
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.8, 7.4, 60.0)
        self.print_result(result)

        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.5, 9.25, 60.0)
        self.print_result(result)

        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.0, 1.0, 60.0)
        self.print_result(result)

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
