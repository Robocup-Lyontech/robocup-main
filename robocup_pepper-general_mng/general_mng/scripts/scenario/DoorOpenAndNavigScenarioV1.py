__author__ = 'Jacques Saraydaryan'
from AbstractScenario import AbstractScenario

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation


class DoorOpenAndNavigScenarioV1(AbstractScenario):
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self, config):
        self.configuration_ready = False
        self.init_scenario(config)

    def init_scenario(self, config):
        self.lt_perception = LTPerception()
        self.lt_navigation = LTNavigation()

        if self.lt_perception.configurationReady == True and self.lt_navigation.configurationReady == True:
            self.configurationReady = True

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