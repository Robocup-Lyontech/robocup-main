__author__ = 'Benoit Renault'
import rospy
import yaml

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus

############ WIP imports
import json
import os
import time
import qi


class Receptionist2019CPEScenario(AbstractScenario, AbstractScenarioBus, AbstractScenarioAction):

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)

        # TODO START Remove Hardcode !
        #   For this, use config to specify path
        receptionist_scenario_filepath = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json"
        with open(receptionist_scenario_filepath) as data:
            self.scenario = yaml.safe_load(data)
        # TODO END

        # TODO START Move this into HRI ROS Package and replace by action calls
        url = "tcp://" + "192.168.1.189" + ":9559"

        self.session = qi.Session()
        self.session.connect(url)
        self.memory = self.session.service("ALMemory")

        self.api_folder = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-hri_meta/robocup_pepper-hri_python/api"

        with open(os.path.join(self.api_folder, "common.json")) as common_api:
            with open(os.path.join(self.api_folder, "generalManagerToHRI.json")) as gmToHri:
                self.apis = {
                    "common": json.load(common_api),
                    "gmToHRI": json.load(gmToHri)
                }
        # TODO END

    def execute_action(self, step, time_left):
        if step["action"] == "":
            self.memory.raiseEvent(self.apis["gmToHRI"]["stepCompleted"]["ALMemory"], json.dumps({}))
            self.memory.raiseEvent(self.apis["gmToHRI"]["currentStep"]["ALMemory"],
                                   json.dumps({"actionId": step['id']}))
        else:
            obj = step["arguments"]
            # Kind of a hack : add step id to action arguments to be sent to local manager to avoid sending entire step
            obj["actionId"] = step['id']
            self.memory.raiseEvent(self.apis["gmToHRI"]["currentAction"]["ALMemory"], json.dumps(obj))

            if step["action"] == "goTo":
                if time_left <= 0.0:
                    rospy.logerr("Not enough time to navigate !")
                else:
                    self.sendNavOrderAction("NP", "CRRCloseToGoal", step["arguments"]["interestPoint"], time_left)
                self.wait_for_local_manager()
            elif step["action"] == "wait":
                # self.wait(step["arguments"]["time"])
                self.wait_for_local_manager()
            elif step["action"] == "askOpenDoor":
                self.wait_for_local_manager()
            elif step["action"] == "askName":
                self.wait_for_local_manager()
                guests = self.memory.getData(self.apis["common"]["AL_VALUE"]["guests"])
                if "John" in guests:
                    john_name = guests["John"]["name"]
                elif "Guest1" in guests:
                    guest1_name = guests["Guest1"]["name"]
                elif "Guest2" in guests:
                    guest2_name = guests["Guest2"]["name"]
                print(str(guests))
            elif step["action"] == "askDrink":
                self.wait_for_local_manager()
            elif step["action"] == "askAge":
                self.wait_for_local_manager()
            elif step["action"] == "askToFollow":
                self.wait_for_local_manager()
            elif step["action"] == "confirm":
                self.wait_for_local_manager()
                if self.hri_work_status == "confirm":
                    self.current_step_pointer -= 1
                    return
            elif step["action"] == "detectHuman":
                self.wait_for_local_manager()  # TODO Check API
            elif step["action"] == "pointTo":
                self.lookAtObject(step["arguments"]["what"], 100.0)
            elif step["action"] == "presentPerson":
                self.wait_for_local_manager()
            elif step["action"] == "find":
                self.wait_for_local_manager()  # TODO Check API
            elif step["action"] == "seatGuest":
                self.wait_for_local_manager()
            elif step["action"] == "finishScenario":
                self.is_scenario_finished = True
            else:
                text = "Action '{action_name}' is not an expected action name".format(action_name=step["action"])
                raise KeyError(text)



        # By default, go to next step
        self.current_step_pointer = self.current_step_pointer + 1

    def wait_for_local_manager(self):
        print("Doing nothing until local manager is done...")
        while not self.hri_work_done:
            time.sleep(0.1)
        self.hri_work_done = False

    def wait(self, duration):
        print("Waiting {duration} seconds...".format(duration=duration))
        time.sleep(duration)
        return None

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self.scenario["name"]))

        # Communicate current scenario to local manager
        self.memory.raiseEvent(self.apis["gmToHRI"]["currentScenario"]["ALMemory"],
                               json.dumps({"scenario": self.scenario}))

        # Initialize step listener
        self.hri_work_done = False
        self.hri_work_status = "done"

        self.callback_ids = {}
        self.attach_event(self.apis["gmToHRI"]["actionComplete"]["ALMemory"], self.handle_hri_job_complete)

        self.is_scenario_finished = False
        self.current_step_pointer = 0
        self.current_step_id = ""

        # TODO START Move this into HRI ROS Package and replace by action calls
        # Start the timer in local manager
        self.memory.raiseEvent(self.apis["gmToHRI"]["timerState"]["ALMemory"],
                               json.dumps({"state": self.apis["gmToHRI"]["timerState"]["state"]["on"]}))
        # TODO END

        self.steps = self.scenario["steps"]
        actual_step_duration = 0.0
        while not self.is_scenario_finished:
            step = self.steps[self.current_step_pointer]
            self.current_step_id = step["id"]

            start_step_time = time.time()

            time_left = step["eta"] if (step["eta"] != 0.0 and step["eta"] != "") else time_left - actual_step_duration

            rospy.loginfo("Action {order} ({id}): {name}, time left is {time_left} seconds".format(
                order=self.current_step_pointer, id=step["id"], name=step["name"], time_left=time_left))

            self.execute_action(step, time_left)

            end_step_time = time.time()

            actual_step_duration = end_step_time - start_step_time

            rospy.loginfo("Step took {duration} seconds to complete.".format(duration=actual_step_duration))

        rospy.loginfo("""
        ######################################
        Finished executing the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self.scenario["name"]))
        # Reset Scenario !
        self.is_scenario_finished = False

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

    def handle_hri_job_complete(self, return_json):
        return_dict= json.loads(return_json)

        if "ok" in return_dict:
            if return_dict["ok"] == self.current_step_id:
                self.hri_work_done = True
                rospy.logerr("Signaled action complete for {hri_action} and waiting for {actual_action}!".format(
                    hri_action=return_dict["ok"], actual_action=self.current_step_id)
                )
            else:
                rospy.logerr("Tried to signal action complete for {hri_action} but waiting for {actual_action}!".format(
                    hri_action=return_dict["ok"], actual_action=self.current_step_id)
                )
        if "error" in return_dict:
            if return_dict["error"] == "confirm":
                self.hri_work_status = "confirm"
        else:
            self.hri_work_status = "done"

    def attach_event(self, event_name, callback):
        try:
            self.callback_ids[event_name] = self.memory.subscriber(event_name)
            self.callback_ids[event_name].signal.connect(callback)
            print("Attached event {event_name} to {callback_name}".format(
                event_name=event_name, callback_name=callback.__name__))
        except Exception as ex:
            print("Something went wrong while attaching event {event_name}: {exception}".format(
                event_name=event_name, exception=str(ex)))

    def find_in_steps_by_id(self, steps_array, step_id):
        steps = [step for step in steps_array if step["id"] == step_id]
        return steps[0] if len(steps) == 1 else None
