#!/usr/bin/env python

import rospy
import yaml
import threading
from std_msgs.msg import String
import importlib
from os import walk, path


class GeneralManager:
    """
    Command Samples:
        rostopic pub /gm_start std_msgs/String "data: '<YOUR_SCENARIO_NAME (NOT FULL PATH, NO EXTENSION !)>'"
    """

    STARTED_STATUS = "START"
    PENDING_STATUS = "PENDING"

    _current_scenario = None
    # FIXME ADD ROS PARAM
    _current_status = PENDING_STATUS

    def __init__(self, config):
        rospy.init_node('general_manager')

        dirname_GM = os.path.dirname(__file__)

        # load face files form data directory
        self.scenarios_data_folder = os.path.join(dirname_GM,config["scenarios_data_folder"])
        # rospy.loginfo("DATA FOLDER : %s",str(self.scenarios_data_folder))
        self.default_scenario_name = '' if "default_scenario_name" not in config else config["default_scenario_name"]

        self._start_sub = rospy.Subscriber("/gm_start", String, self.gm_start_callback)

        if self.default_scenario_name != '':
            self._current_scenario = self.load_scenario(self.default_scenario_name)

        # self._current_scenario.start_scenario()  # UNCOMMENT FOR DEBUG PURPOSES

    def load_scenario(self, scenario_name):
        """
        Load the choosen scenario. 

        :param scenario_name: Choosen scenario name
        :type scenario_name: string
        """
        scenario_path = self.search_scenario_path_by_filename(scenario_name)

        if scenario_path is None:
            scenario_path = self.search_scenario_path_by_content(scenario_name)

        if scenario_path is None:
            rospy.logerr("{class_name} : Could not find a scenario data file in folder {0} matching name {1}.".format(
                self.scenarios_data_folder, scenario_name, class_name=self.__class__.__name__))
            return None

        #try:
        with open(scenario_path) as json_scenario:
            scenario_data = yaml.safe_load(json_scenario)

        if "imports" in scenario_data:
            for key, rel_path in scenario_data["imports"].items():
                with open(path.join(self.scenarios_data_folder, rel_path)) as imported_json_file:
                    scenario_data["imports"][key] = yaml.safe_load(imported_json_file)

        try:
            scenario_module = importlib.import_module("scenario." + scenario_name)
            try:
                scenario_class = getattr(scenario_module, scenario_name)
                try:
                    return scenario_class(scenario_data, self.scenarios_data_folder)
                except Exception as e:
                    rospy.logerr("{class_name} : Scenario data and class were loaded properly but scenario object could not be created.".format(class_name=self.__class__))
                    rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + e.__class__.__name__ + ": " + e.message)
            except AttributeError as e:
                rospy.logerr("{class_name} : Could not find a class in scenario python module with name ".format(class_name=self.__class__.__name__) + scenario_name)
                rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + e.__class__.__name__ + ": " + e.message)
        except ImportError as e:
            rospy.logerr("{class_name} : Could not find a scenario python module with name ".format(class_name=self.__class__.__name__) + scenario_name)
            rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + e.__class__.__name__ + ": " + e.message)
        #except yaml.YAMLError as e:
        #    rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + "File {0} is not properly formatted JSON or YAML. Please fix it or remove it from the folder.")
        #    rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + e.__class__.__name__ + ": " + e.message)

    def execute_current_scenario(self):
        """
        Execute the loaded scenario if the scenario configuration is succesfully set. 
        """
        if self._current_scenario is None:
            rospy.logwarn("{class_name} : No scenario is currently loaded. Maybe wait a bit more to send a START command or load a new one.".format(class_name=self.__class__.__name__))
            return
        else:
            if self._current_scenario.configuration_ready:
                self._current_scenario.start_scenario()
            else:
                rospy.logwarn("{class_name} : Currently loaded scenario is not ready. Maybe wait a bit more to send a START command.".format(class_name=self.__class__.__name__))

    def gm_start_callback(self, msg):
        """
        Callback function for /gm_start subscriber. Gets the choosen scenario in msg.data.
        If the choosen scenario isn't started yet, loads the choosen scenario and runs it.

        :param msg: /gm_start topic data containing the name of a scenario
        :type msg: std_msgs/String
        """
        lock = threading.Lock()
        lock.acquire()
        if self._current_status == self.STARTED_STATUS:
            rospy.loginfo('{class_name} : SCENARIO IS ALREADY STARTED...'.format(class_name=self.__class__.__name__))
            return
        self._current_status = self.STARTED_STATUS
        lock.release()

        rospy.loginfo('{class_name} : START CMD: '.format(class_name=self.__class__.__name__) + msg.data)
        if msg.data == 'START':
            self.execute_current_scenario()
        else:
            scenario_name = msg.data
            self._current_scenario = self.load_scenario(scenario_name)
            self.execute_current_scenario()
        self._current_status = self.PENDING_STATUS
        self._current_scenario = None
        rospy.loginfo('{class_name} : -----------------------------------END CURRENT SCENARIO-----------------------'.format(class_name=self.__class__.__name__))

    def search_scenario_path_by_filename(self, scenario_name):
        """
        Searches a scenario JSON file with the same name as the choosen scenario name.

        :param scenario_name: scenario name
        :type scenario_name: string
        """
        for (dirpath, dirnames, filenames) in walk(self.scenarios_data_folder):
            for filename in filenames:
                if scenario_name == path.splitext(filename)[0]:
                    return path.join(dirpath, filename)
        return None

    def search_scenario_path_by_content(self, scenario_name):
        """
        Searches a scenario JSON file which contains the name of the choosen scenario name.

        :param scenario_name: scenario name
        :type scenario_name: string
        """
        for (dirpath, dirnames, filenames) in walk(self.scenarios_data_folder):
            for filename in filenames:
                file_path = path.join(dirpath, filename)
                with open(file_path) as cur_file:
                    try:
                        file_data = yaml.safe_load(cur_file)
                        if 'name' in file_data and file_data['name'] == scenario_name:
                            return file_path
                    except yaml.YAMLError as e:
                        rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + "File {0} is not properly formatted JSON or YAML. Please fix it or remove it from the folder.".format(file_path))
                        rospy.logerr("{class_name} : ".format(class_name=self.__class__.__name__) + e.__class__.__name__ + ": " + e.message)
        return None


if __name__ == '__main__':
    import os

    dirname = os.path.dirname(__file__)
    filepath = os.path.join(dirname, '../config/common_gm.yaml')

    with open(filepath) as data:
        gm_config = yaml.safe_load(data)

    gm = GeneralManager(gm_config)

    rospy.spin()
