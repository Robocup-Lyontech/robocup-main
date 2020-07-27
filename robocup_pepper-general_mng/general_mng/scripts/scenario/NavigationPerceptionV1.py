__author__ = 'Jacques Saraydaryan'
import rospy
from AbstractScenario import AbstractScenario

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTHriManager import LTHriManager


class NavigationPerceptionV1(AbstractScenario):
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self, config):
        self.configuration_ready = False
        self.init_scenario(config)

    def init_scenario(self, config):
        self.lt_perception = LTPerception()
        self.lt_navigation = LTNavigation()
        self.lt_hri = LTHriManager("192.168.1.222", 9559, "HRI_MNG_")

        if self.lt_perception.configurationReady == True and self.lt_navigation.configurationReady == True and self.lt_hri.configurationReady == True:
            self.configurationReady = True

    def start_scenario(self):
        self.print_name(self.__name__)

        # Wait door opening
        result = self.lt_perception.wait_for_door_to_open()
        self.print_result(result)


        # start navigation A
        # result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.8, 7.4, 60.0)
        # self.print_result(result)

        name = "BIG_HERO"

        # learn a people Face associates to a name
        result = self.lt_perception.learn_people_meta_from_img_topic(name, 10.0)
        self.print_result(result)

        # start navigation B
        # result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 8.8, 1.25, 60.0)
        # self.print_result(result)

        # detect people face
        result = self.lt_perception.get_people_name_from_img_topic(10.0)
        self.print_result(result)
        try:
            rospy.loginfo(result.payload.peopleNames)
            rospy.loginfo(result.payload.peopleNamesScore)
        except Exception as e:
            rospy.logwarn("payload is not compliant:"+str(e))

        # start navigation C
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.0, 1.0, 60.0)
        self.print_result(result)
