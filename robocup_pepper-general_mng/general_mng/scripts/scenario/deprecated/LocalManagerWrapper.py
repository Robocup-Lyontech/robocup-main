import json
import qi
import rospy
import threading
import time
import uuid
from actionlib_msgs.msg import GoalStatus


class LocalManagerWrapper:
    GOAL = "goal"
    RESULT = "result"

    def __init__(self, ip_address, tcp_port, prefix):
        self._ip = ip_address
        self._port = tcp_port
        self._session = None
        self._memory = None
        self._service = None

        self._prefix = prefix
        self._almemory_channels = dict()
        self._results_buffer = dict()

        while not self._configure_naoqi(ip_address, tcp_port) and not rospy.is_shutdown():
            time.sleep(0.5)

        # TODO Add Heartbeat

        rospy.loginfo("{class_name}: Configuration finished.".format(class_name=self.__class__))

    def _configure_naoqi(self, ip, port):
        rospy.loginfo("{class_name}: Connecting to ALMemory...".format(class_name=self.__class__))
        self._session = qi.Session()
        try:
            self._session.connect("tcp://" + ip + ":" + str(port))
            self._memory = self._session.service("ALMemory")
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip {ip_address} on port {port}. "
                         "\n Please check your script arguments.".format(ip_address=ip, port=port))
            return False
        rospy.loginfo("{class_name}: Connected to ALMemory".format(class_name=self.__class__))
        return True

    def _init_channels(self, action_name):
        channels = {}

        for channel_type in [
            self.GOAL,
            self.RESULT
        ]:
            channel = self._get_almemory_address(self._prefix, action_name, channel_type)
            channels[channel_type] = channel
            self._memory.insertData(channel, json.dumps(dict()))

        self._almemory_channels[action_name] = channels

    def _naoqi_callback(self, payload):
        rospy.loginfo("{class_name}: Naoqi callback called. Payload: {payload}".format(
            class_name=self.__class__, payload=payload))
        payload_dict = json.loads(payload)
        lock = threading.Lock()
        lock.acquire()
        self._results_buffer[payload_dict["id"]] = payload_dict
        lock.release()

    def _is_result_received(self, goal_id):
        lock = threading.Lock()
        lock.acquire()
        result_received = (goal_id in self._results_buffer)
        lock.release()
        return result_received

    @staticmethod
    def _get_almemory_address(prefix, action_name, topic_type):
        return "{0}/{1}/{2}".format(prefix, action_name, topic_type)

    @staticmethod
    def _is_timeout_over(start_time, timeout):
        if timeout <= 0.0:
            return False
        return time.time() - start_time > timeout

    def _execute_request(self, action, payload, timeout):
        start_time = time.time()

        rospy.loginfo(
            "{class_name}: Received action <{action_name}> for local manager. Arguments: \n{args}".format(
                class_name=self.__class__, action_name=action, args=payload))

        # If action never been sent before, initialize communication channels
        if action not in self._almemory_channels:
            try:
                self._init_channels(action)
            except Exception as e:
                rospy.logerr("{class_name}: Could not initialize channels in ALMemory: {error}. ABORT !".format(
                    class_name=self.__class__, error=e))
                return GoalStatus.ABORTED, None

        goal_channel = self._almemory_channels[action][self.GOAL]
        result_channel = self._almemory_channels[action][self.RESULT]

        goal_id = json.loads(payload)["id"]

        # Start listening on result channel
        try:
            result_channel_subscriber = self._memory.subscriber(result_channel)
            result_channel_subscriber.signal.connect(self._naoqi_callback)
        except Exception as e:
            rospy.logerr("{class_name}: Could not initialize subscriber to result: {error}. ABORT !".format(
                class_name=self.__class__, error=e))
            return GoalStatus.ABORTED, None

        # Send goal on goal channel if channel as soon as channel is free
        try:
            goal_on_channel = json.loads(self._memory.getData(goal_channel))
            while goal_on_channel:
                if self._is_timeout_over(start_time, timeout):
                    rospy.loginfo("{class_name}: Action {action_name} has been preempted, goal not sent yet.".format(
                        class_name=self.__class__, action_name=action))
                    return GoalStatus.PREEMPTED, None
                goal_on_channel = json.loads(self._memory.getData(goal_channel))
                time.sleep(0.1)

            self._memory.raiseEvent(goal_channel, payload)
        except Exception as e:
            rospy.logerr("{class_name}: Could not send goal: {error}. ABORT !".format(
                class_name=self.__class__, error=e))
            return GoalStatus.ABORTED, None

        # Wait for result to be received
        try:
            while not self._is_result_received(goal_id) and not rospy.is_shutdown():
                if self._is_timeout_over(start_time, timeout):
                    rospy.loginfo(
                        "{class_name}: Action {action_name} has been preempted, result not received yet.".format(
                            class_name=self.__class__, action_name=action))
                    return GoalStatus.PREEMPTED, None
                time.sleep(0.1)

            # Consume result
            self._memory.insertData(result_channel, json.dumps(dict()))
        except Exception as e:
            rospy.logerr("{class_name}: Could not receive result: {error}. ABORT !".format(
                class_name=self.__class__, error=e))
            return GoalStatus.ABORTED, None

        # Return result for action
        lock = threading.Lock()
        lock.acquire()
        result = self._results_buffer[goal_id]
        lock.release()
        return GoalStatus.SUCCEEDED, result

    ####################################################################################################################
    # Start of API between GeneralManager and HRI
    ####################################################################################################################

    def ask_name(self, speech, people, timeout):
        """
        Start the view 'askName'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param names: a list of names that can be chosen by operators
        :type names: list
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        names = []
        for person in people:
            names.append(person['name'])

        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'names': names
            }
        })
        status, result = self._execute_request("askName", payload, timeout)
        if status == GoalStatus.SUCCEEDED and 'name' in result:
            return status, result['name']
        else:
            return status, "ERROR NAME"

    def ask_drink(self, speech, drinks, timeout):
        """
        Start the view 'askDrink'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param drinks: a list of drinks that can be chosen by operators
        :type drinks: list
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'drinks': drinks
            }
        })
        status, result = self._execute_request("askDrink", payload, timeout)
        if status == GoalStatus.SUCCEEDED and 'drink' in result:
            return status, result['drink']
        else:
            return status, "ERROR DRINK"

    def generic(self, timeout, speech, image=None, video=None, p_list=None):
        """
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout int
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech dict
        :param image: the path for TODO
        :type image dict
        :param video:
        :type video dict
        :param p_list:
        :type p_list list
        :return:
        """
        data = {}

        if image and type(image) == dict:
            if "pathOnTablet" in image and "alternative":
                data['image'] = image

        if video and type(video) == dict:
            if "pathOnTablet" in video and "alternative" in video:
                data['video'] = video

        if p_list and type(p_list) == list:
            data['list'] = p_list

        if speech and type(speech) == dict:
            data['speech'] = speech

        status, result = self._execute_request("generic", json.dumps(data), timeout)
        if status == GoalStatus.SUCCEEDED and 'drink' in result:
            return status, result['drink']
        else:
            return status, "ERROR GENERIC"

    def ask_age(self, speech, timeout):
        """
        Start the view 'askAge'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self._execute_request("askAge", payload, timeout)
        return status

    def confirm(self, speech, timeout):
        """
        Start the view 'confirm'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self._execute_request("confirm", payload, timeout)
        if status == GoalStatus.SUCCEEDED and 'confirm' in result:
            return status, result['confirm']
        else:
            return status, False

    def detail_drinks(self, timeout):
        raise NotImplementedError()

    def present_person(self, speech, name_to_present, drink_to_present, names_present_to, timeout):
        """
        Present one person and its favorite drink to a list of other persons

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param name_to_present:
        :type name_to_present: string
        :param drink_to_present:
        :type drink_to_present: drink json object (string)
        :param names_present_to:
        :param names_present_to: string[]
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'who': {
                    'name': name_to_present,
                    'drink': drink_to_present
                },
                'to': names_present_to
            }
        })
        status, result = self._execute_request("presentPerson", payload, timeout)
        return status

    def go_to(self, speech, location, timeout):
        """
        Start the view 'goTo'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'location': location
            }
        })
        status, result = self._execute_request("goTo", payload, timeout)
        return status

    def seat_guest(self, speech, timeout):
        """
        Start the view 'seatGuest'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param guest: name of the guest to seat
        :type guest: string
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
            }
        })
        status, result = self._execute_request("seatGuest", payload, timeout)
        return status

    def ask_to_follow(self, speech, location, timeout):
        """
        Start the view 'askToFollow'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'location': location
            }
        })
        status, result = self._execute_request("askToFollow", payload, timeout)
        return status

    def main_menu(self, speech, scenarios, timeout):
        """
        Start the view 'mainMenu'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param scenarios: list of scenario names to display
        :type scenarios: string[]
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'scenarios': scenarios
            }
        })
        status, result = self._execute_request("mainMenu", payload, timeout)
        return status

    def wait(self, speech, waiting_time, timeout):
        """
        Start the view 'wait'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param waiting_time: time for local manager to wait
        :type waiting_time: float
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'time': waiting_time,
                'speech': speech
            }
        })
        status, result = self._execute_request("wait", payload, timeout)
        return status

    def ask_open_door(self, speech, timeout):
        """
        Start the view 'askOpenDoor'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self._execute_request("askOpenDoor", payload, timeout)
        return status

    def call_human(self, speech, waiting_time, timeout):
        """
        Start the view 'callHuman'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param waiting_time: time for local manager to wait
        :type waiting_time: float
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'time': waiting_time
            }
        })
        status, result = self._execute_request("callHuman", payload, timeout)
        return status

    def show_video(self, speech, video_url, timeout):
        """
        Start the view 'showVideo'

        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param video_url: relative path to video
        :type video_url: string
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'video': video_url
            }
        })
        status, result = self._execute_request("showVideo", payload, timeout)
        return status

    def find_available_drinks(self, timeout):
        raise NotImplementedError()

    def open_door(self, timeout):
        raise NotImplementedError()

    def find_who_wants_drinks(self, timeout):
        raise NotImplementedError()

    def serve_drinks(self, timeout):
        raise NotImplementedError()

    # Timeboard API

    def timeboard_send_step_done(self, step_indexes, timeout):
        """
        Set done a list of steps

        :param step_indexes: A list of indexes of the steps
        :type step_indexes: list
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'indexes': [step_indexes] if type(step_indexes) == int else step_indexes
            }
        })
        status, result = self._execute_request("stepDone", payload, timeout)
        return status

    def timeboard_send_steps_list(self, steps, scenario_name, timeout):
        """
        Change all steps in timeboard

        :param steps: The list of steps extracted from the scenario json file
        :type steps: list
        :param scenario_name: name of the scenario to which the steps belong
        :type scenario_name: string
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        step_list = []
        step_id_to_index = {}
        index = 0
        for step in steps:
            if not step['action']:
                step_list.append({'name': step['name'], 'eta': step['eta'], 'id': step['id']})
                step_id_to_index[step['id']] = index
                index += 1

        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'scenarioName': scenario_name,
                'stepsList': step_list
            }
        })
        status, result = self._execute_request("stepsList", payload, timeout)
        return status, step_id_to_index

    def timeboard_send_step_skipped(self, step_indexes, timeout):
        """
        Set skipped a list of steps

        :param step_indexes: A list of indexes of the steps
        :type step_indexes: list
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'indexes': step_indexes
            }
        })
        status, result = self._execute_request("stepSkipped", payload, timeout)
        return status

    def timeboard_set_current_step(self, step_index, timeout):
        """
        Set current a steps

        :param step_index: the index of the step
        :type step_index: int
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'index': step_index
            }
        })
        status, result = self._execute_request("currentStep", payload, timeout)
        return status

    def timeboard_set_timer_state(self, activate, timeout):
        """
        Sets local manager timer state so that timeboard actually counts the passing of time

        :param activate: target state of the timer
        :type activate: bool
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        LOCAL_MANAGER_TIMER_ON_MSG = 'TOGGLE_TIMER/ON'
        LOCAL_MANAGER_TIMER_OFF_MSG = "TOGGLE_TIMER/OFF"

        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'state': LOCAL_MANAGER_TIMER_ON_MSG if activate else LOCAL_MANAGER_TIMER_OFF_MSG
            }
        })
        status, result = self._execute_request("toggleTimer", payload, timeout)
        return status

    # TODO Add Timer start function

    ####################################################################################################################
    # End of API between GeneralManager and HRI
    ####################################################################################################################
