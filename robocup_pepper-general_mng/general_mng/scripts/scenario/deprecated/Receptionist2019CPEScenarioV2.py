__author__ = 'Benoit Renault'
import rospy

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus

import json
import os
import threading
import time

import qi


class Receptionist2019CPEScenarioV2(AbstractScenario, AbstractScenarioBus, AbstractScenarioAction):

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)

        self.url = "tcp://" + "192.168.1.189" + ":9559"
        self.session = qi.Session()
        self.session.connect(self.url)
        self.memory = self.session.service("ALMemory")

        self.api_folder = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-hri_meta/robocup_pepper-hri_python/api"

        with open(os.path.join(self.api_folder, "common.json")) as common_api:
            with open(os.path.join(self.api_folder, "generalManagerToHRI.json")) as gmToHri:
                self.apis = {
                    "common": json.load(common_api),
                    "gmToHRI": json.load(gmToHri)
                }

        self.callback_ids = {}
        self.steps = []
        self.curr_step_ind = 0

        self.ros_work_done = False
        self.hri_work_done = False
        self.hri_work_return_value = ""

        receptionist_scenario_filepath = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json"
        with open(receptionist_scenario_filepath) as data:
            self.scenario = json.load(data)

        threading.Thread(target=self.gm_heartbeat).start()

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self.scenario["name"]))

        self.steps = self.scenario["steps"]
        # Change the current scenario
        self.memory.raiseEvent(self.apis["gmToHRI"]["currentScenario"]["ALMemory"],
                               json.dumps({"scenario": self.scenario}))

        # Initialize step listener
        self.attach_event(self.apis["gmToHRI"]["actionComplete"]["ALMemory"], self.handle_hri_job_complete)

        # Start the timer
        self.memory.raiseEvent(self.apis["gmToHRI"]["timerState"]["ALMemory"],
                               json.dumps({"state": self.apis["gmToHRI"]["timerState"]["state"]["on"]}))

        if self.curr_step_ind < len(self.steps):
            self.next_step()

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

        AbstractScenarioAction.configure_intern(self)

    def gm_heartbeat(self):
        while True:
            self.memory.raiseEvent(self.apis["common"]["generalManagerHeartbeat"]["ALMemory"],
                                   json.dumps({'time': time.time()}))
            time.sleep(1)

    def simulate_ros_work(self, time_for_work):
        time.sleep(time_for_work)
        self.ros_work_done = True
        self.handle_common_complete("{}")

    def handle_common_complete(self, return_dict_json):
        if self.ros_work_done and self.hri_work_done:
            return_dict = json.loads(return_dict_json)
            print(return_dict)
            if "error" in return_dict:
                if return_dict["error"] == "confirm":
                    self.curr_step_ind -= 1
            elif "ok" in return_dict:
                if return_dict["ok"] == self.steps[self.curr_step_ind][id]:
                    rospy.loginfo("Signaled action complete for {hri_action} and waiting for {actual_action}!".format(
                        hri_action=return_dict["ok"], actual_action=self.steps[self.curr_step_ind][id])
                    )
                    self.curr_step_ind += 1
                else:
                    rospy.logerr(
                        "Tried to signal action complete for {hri_action} but waiting for {actual_action}!".format(
                        hri_action=return_dict["ok"], actual_action=self.steps[self.curr_step_ind][id])
                    )
            elif not return_dict:
                self.curr_step_ind += 1

            self.hri_work_done = False
            self.hri_work_return_value = ""

            self.next_step()

    def handle_hri_job_complete(self, value):
        self.hri_work_done = True
        self.hri_work_return_value = value
        rospy.loginfo("HRI Job completed")

    def next_step(self):
        if len(self.steps) > self.curr_step_ind:

            curr_step = self.steps[self.curr_step_ind]
            self.ros_work_done = False

            if curr_step["action"] == "":
                rospy.loginfo("New major task: {task_name}".format(task_name=curr_step["name"]))

                self.memory.raiseEvent(self.apis["gmToHRI"]["stepCompleted"]["ALMemory"], json.dumps({}))

                rospy.loginfo("Send current action, actionId: {action_id}".format(action_id=curr_step['id']))
                self.memory.raiseEvent(self.apis["gmToHRI"]["currentStep"]["ALMemory"],
                                       json.dumps({"actionId": curr_step['id']}))

                self.hri_work_done = self.ros_work_done = True
                self.handle_common_complete("{}")
            else:
                obj = curr_step["arguments"]
                obj["actionId"] = curr_step['id']

                rospy.loginfo("Send action with name: {action_name}".format(action_name=curr_step['name']))
                self.memory.raiseEvent(self.apis["gmToHRI"]["currentAction"]["ALMemory"], json.dumps(
                    obj
                ))

                if curr_step["action"] in ["humanDetection", "goTo", "pointTo", "find", "closeHand", "openHand",
                                           "checkBag", "holdBag", "finishScenario"]:
                    if curr_step["action"] == "goTo":
                        self.sendNavOrderAction("NP", "CRRCloseToGoal", curr_step["arguments"]["interestPoint"], 100.0)
                        self.ros_work_done = True
                        self.handle_common_complete("{}")
                        # self.sendNavOrderAction("NP", "CRRCloseToGoal", curr_step["arguments"]["interestPoint"], time_left)
                    elif curr_step["action"] == "finishScenario":
                        rospy.loginfo("""
                                ######################################
                                Finished executing the {scenario_name} Scenario...
                                ######################################
                                """.format(scenario_name=self.scenario["name"]))
                    else:
                        rospy.loginfo("Simulate {action} by waiting 2 seconds.".format(action=curr_step["action"]))
                        self.simulate_ros_work(2)
                else:
                    rospy.loginfo("ROS for 1 seconds")
                    self.simulate_ros_work(1)

            self.wait_for_local_manager()

            self.handle_common_complete(self.hri_work_return_value)

    def wait_for_local_manager(self):
        while not self.hri_work_done:
            time.sleep(0.1)

    def attach_event(self, event_name, callback):
        try:
            self.callback_ids[event_name] = self.memory.subscriber(event_name)
            self.callback_ids[event_name].signal.connect(callback)
            rospy.loginfo("Attached event: {event_name} to: {callback_name}".format(
                event_name=event_name, callback_name=callback.__name__))
        except Exception as ex:
            rospy.logerr("Something went wrong while attaching event {event_name}: {exception}.".format(
                event_name=event_name, exception=str(ex)))
