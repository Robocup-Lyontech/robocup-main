__author__ = 'Jacques Saraydaryan'
from abc import abstractmethod
import rospy
from meta_lib.LTServiceResponse import LTServiceResponse


class AbstractScenario:
    """
    Abstract Scenario class
    Any scenario must inherit from it and implement the abstract methods.
    Any scenario class name must be the same as the scenario data filename (without extension of course).
    """
    @abstractmethod
    def start_scenario(self):
        """
        This method must contain the scenario logic (the sequence/organization) of actions that need to be done.
        """
        pass

    @abstractmethod
    def init_scenario(self, scenario_data):
        """
        This method must reset the scenario configuration variables and
        :param config:
        :type config:
        """
        pass

    @staticmethod
    def print_result(data):
        """
        Utility function to display the result of an action.
        :param data: the action data
        """
        if data.status == LTServiceResponse.FAILURE_STATUS:
            rospy.logwarn(data)
        else:
            rospy.loginfo(data)

    @staticmethod
    def print_name(name):
        """
        Utility function to display the scenario name
        :param name: the scenario name
        :type name: str
        """
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting Scenario: {0} ...".format(name))
        rospy.loginfo("######################################")
