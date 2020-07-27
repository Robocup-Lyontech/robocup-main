
__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse
from actionlib_msgs.msg import GoalStatus
from HriManager.msg import GmToHriAction, GmToHriGoal, GmToHriFeedback

import actionlib
import rospy

import time
import threading
import json
import uuid

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
class LTHriManagerPalbator(LTAbstract):

    GOAL = "goal"
    RESULT = "result"

    _enableNavAction = True

    def __init__(self,name_HRIAction):
        """
        Initializes the LTHriManager API for Palbator. It will deal with every function related to Palbator's HRI.
        """
        self.client_action_GmToHri=actionlib.SimpleActionClient(name_HRIAction,GmToHriAction)

        #Inform configuration is ready
        self.configurationReady = True

    def _execute_request(self, payload, timeout):
        """
        Send the goal to HRIManager as an ActionClient and wait for result.

        :param payload: json converted in string containing action parameters
        :type payload: string
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        start_time = time.time()
        rospy.loginfo("{class_name} : REQUEST RECEIVED FROM SCENARIO".format(class_name=self.__class__.__name__))
        goal=GmToHriGoal(payload)
        rospy.loginfo("{class_name} : GOAL TO SEND ".format(class_name=self.__class__.__name__)+str(goal))
        self.client_action_GmToHri.send_goal(goal)
        rospy.loginfo("{class_name} : GOAL SENT TO ACTION".format(class_name=self.__class__.__name__))
        

        # Wait for result to be received
        try:
            rospy.loginfo("{class_name} : WAITING FOR ACTION RESULT ".format(class_name=self.__class__.__name__))
            self.client_action_GmToHri.wait_for_result()
            result_action=self.client_action_GmToHri.get_result()
            rospy.loginfo("{class_name} : GOT RESULT FROM ACTION".format(class_name=self.__class__.__name__))
            resultat=result_action.Gm_To_Hri_output
            json_resultat=json.loads(resultat)
            rospy.loginfo("{class_name}  JSON RESULT: ".format(class_name=self.__class__.__name__)+str(json_resultat))


        except Exception as e:
            rospy.logerr("{class_name}: Could not receive result: {error}. ABORT !".format(
                class_name=self.__class__.__name__, error=e))
            return GoalStatus.ABORTED, None

        # Return result for action
        lock = threading.Lock()
        lock.acquire()
        result=json_resultat
        lock.release()
        return GoalStatus.SUCCEEDED, result

    def restart_hri(self,timeout):
        """
        Restart the HRI
        """

        payload = json.dumps({
            'timestamp': time.time(),
            "action": "RESTART"
        })
        status, result = self._execute_request(payload, timeout)
        return status,result

    def timeboard_send_steps_list(self, steps, scenario_name, timeout):
        """
        Send all steps to HRIManager

        :param steps: The list of steps extracted from the scenario json file
        :type steps: list
        :param scenario_name: name of the scenario to which the steps belong
        :type scenario_name: string
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        
        payload = json.dumps({
            'timestamp': time.time(),
            "action": "stepsList",
            "scenario": scenario_name,
            "stepsList": steps,
            })
        status, result = self._execute_request(payload, timeout)
        return status,result
    
    def timeboard_set_current_step(self, step_index, timeout):
        """
        Set the current step

        :param step_index: the index of the step
        :type step_index: int
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
      
        payload = json.dumps({
                'timestamp': time.time(),
                'action': "currentStep",
                "stepIndex": step_index,
            })
        status, result = self._execute_request(payload, timeout)
        return status, result
    
    def timeboard_set_current_step_with_data(self, step_index, data, timeout):
        """
        Set the current step

        :param step_index: the index of the step
        :type step_index: int
        :param data: the additional data to use with the current step
        :type data: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        
        payload = json.dumps({
                'timestamp': time.time(),
                'action': "currentStep",
                "stepIndex": step_index,
                "data": data
            })
        status, result = self._execute_request(payload, timeout)
        return status, result


class LTHriManager(LTAbstract):
    

    GOAL = "goal"
    RESULT = "result"

    _enableNavAction = True

    def __init__(self, ip_address="127.0.0.1", tcp_port=9559, prefix="HRI_MNG_"):
        import qi
        self.configure_intern(ip_address, tcp_port, prefix)

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self, ip_address, tcp_port, prefix):
        rospy.loginfo("Connecting to Hri_manager action server ... ")

        self._ip = ip_address
        self._port = tcp_port
        self._session = None
        self._memory = None
        self._service = None

        self._prefix = prefix
        self._almemory_channels = dict()
        self._results_buffer = dict()

        start_time = time.time()
        while not self.__configure_naoqi(ip_address, tcp_port) and (time.time()-start_time) < self.LTAbstract.REMOTE_DEVICE_WAIT_TIMEOUT:
                time.sleep(0.5)

    def reset(self):
        self.configure_intern()

    def __configure_naoqi(self, ip, port):
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

    def __init_channels(self, action_name):
        channels = {}

        for channel_type in [
            self.GOAL,
            self.RESULT
        ]:
            channel = self.__get_almemory_address(self._prefix, action_name, channel_type)
            channels[channel_type] = channel
            self._memory.insertData(channel, json.dumps(dict()))

        self._almemory_channels[action_name] = channels



    #######################################
    # HRI MANAGEMENT API
    ######################################
    def generic(self, timeout, speech, image=None, video=None, p_list=None):
        """
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout float
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
        response = LTServiceResponse()
        data = {
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {}}

        if image and type(image) == dict:
            if "pathOnTablet" in image and "alternative":
                data['args']['image'] = image

        if video and type(video) == dict:
            if "pathOnTablet" in video and "alternative" in video:
                data['args']['video'] = video

        if p_list and type(p_list) == list:
            data['args']['list'] = p_list

        if speech and type(speech) == dict:
            data['args']['speech'] = speech

        status, result = self.__execute_request("generic", json.dumps(data), timeout)
        response.process_state(status)
        response.payload = result
        return response

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
        response = LTServiceResponse()
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
        status, result = self.__execute_request("askName", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS and 'name' in result:
            response.msg = " Operation success during ask_name to speech:[%s], people:[%s], name:[%s]" % (
                speech, people,result['name'])
            response.payload = result['name']
            return response
        else:
            response.msg = " Failure HRI Manager ask_name to speech:[%s], people:[%s]" % (
                speech, people)
            return response

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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'drinks': drinks
            }
        })
        status, result = self.__execute_request("askDrink", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS and 'drink' in result:
            response.msg = " Operation success during ask_drink to speech:[%s], drinks:[%s], drink:[%s]" % (
                speech, drinks,result['drink'])
            response.payload = result['drink']
            return response
        else:
            response.msg = " Failure HRI Manager ask_drink to speech:[%s], drinks:[%s]" % (
                speech, drinks)
            return response

    def ask_age(self, speech, timeout):
        """
        Start the view 'askAge'
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self.__execute_request("askAge", payload, timeout)
        # if status == GoalStatus.SUCCEEDED and 'age' in result:
        #     return status, result['age']
        # else:
        #     return status, 666

        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS and 'age' in result:
            response.msg = " Operation success during ask_age to speech:[%s], age:[%s]" % (
                speech, result['age'])
            response.payload = result['age']
            return response
        else:
            response.msg = " Failure HRI Manager ask_age to speech:[%s]" % (
                speech)
            return response


    def confirm(self, speech, timeout):
        """
        Start the view 'confirm'
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self.__execute_request("confirm", payload, timeout)
        # if status == GoalStatus.SUCCEEDED and 'confirm' in result:
        #     return status, result['confirm']
        # else:
        #     return status, False
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS and 'confirm' in result:
            response.msg = " Operation success during confirm to speech:[%s], confirm:[%s]" % (
                speech, result['confirm'])
            response.payload = result['confirm']
            return response
        else:
            response.msg = " Failure HRI Manager confirm to speech:[%s]" % (
                speech)
            return response

    def detail_drinks(self, timeout):
        raise NotImplementedError()

    def introduce(self, speech, name_to_present, drink_to_present, image_to_present, timeout):
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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'who': {
                    'name': name_to_present,
                    'drink': drink_to_present,
                    'image': image_to_present
                }
            }
        })
        status, result = self.__execute_request("presentPerson", payload, timeout)

        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during introduce to speech:[%s], name_to_present:[%s], drink_to_present:[%s]" % (
                speech,name_to_present, drink_to_present)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager introduce to speech:[%s], name_to_present:[%s], drink_to_present:[%s]" % (
                speech,name_to_present, drink_to_present)
            return response

    def go_to(self, speech, location, timeout):
        """
        Start the view 'goTo'
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'location': location
            }
        })
        status, result = self.__execute_request("goTo", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during go_to to speech:[%s], location:[%s]" % (
                speech, location)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager go_to to speech:[%s], location:[%s]" % (
                speech, location)
            return response

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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
            }
        })
        status, result = self.__execute_request("seatGuest", payload, timeout)

        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during seat_guest to speech:[%s]" % (
                speech)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager seat_guest to speech:[%s]" % (
                speech)
            return response


    def ask_to_follow(self, speech, location, timeout):
        """
        Start the view 'askToFollow'
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'location': location
            }
        })
        status, result = self.__execute_request("askToFollow", payload, timeout)

        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during ask_to_follow to speech:[%s], location:[%s]" % (
                speech,location)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager ask_to_follow to speech:[%s], location:[%s]" % (
                speech,location)
            return response

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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'scenarios': scenarios
            }
        })
        status, result = self.__execute_request("mainMenu", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during main_menu to speech:[%s], scenarios:[%s]" % (
                speech, scenarios)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager main_menu to speech:[%s], scenarios:[%s]" % (
                speech, scenarios)
            return response


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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'time': waiting_time,
                'speech': speech
            }
        })
        status, result = self.__execute_request("wait", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS :
            response.msg = " Operation success during wait to speech:[%s], waiting_time:[%s]" % (
                speech, waiting_time)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager wait to speech:[%s], waiting_time:[%s]" % (
                speech, waiting_time)
            return response


    def ask_open_door(self, speech, timeout):
        """
        Start the view 'askOpenDoor'
        :param speech: the text that will be use by the Local Manager for tablet and vocal
        :type speech: dict
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech
            }
        })
        status, result = self.__execute_request("askOpenDoor", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during ask_open_door to speech:[%s]" % (
                speech)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager ask_open_door to speech:[%s]" % (
                speech)
            return response

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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'time': waiting_time
            }
        })
        status, result = self.__execute_request("callHuman", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during call_human to speech:[%s], waiting_time:[%s]" % (
                speech, waiting_time)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager call_human to speech:[%s], waiting_time:[%s]" % (
                speech, waiting_time)
            return response

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
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'speech': speech,
                'video': video_url
            }
        })
        status, result = self.__execute_request("showVideo", payload, timeout)
        response.process_state(status)

        if response.status == LTServiceResponse.SUCCESS_STATUS:
            response.msg = " Operation success during show_video to speech:[%s], video_url:[%s]" % (
                speech, video_url)
            response.payload = result
            return response
        else:
            response.msg = " Failure HRI Manager show_video to speech:[%s], video_url:[%s]" % (
                speech, video_url)
            return response

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
        response = LTServiceResponse()
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
        response = LTServiceResponse()
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
        response.process_state(status)
        response.payload = step_id_to_index
        return response

    def timeboard_send_step_skipped(self, step_indexes, timeout):
        """
        Set skipped a list of steps
        :param step_indexes: A list of indexes of the steps
        :type step_indexes: list
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'indexes': step_indexes
            }
        })
        status, result = self._execute_request("stepSkipped", payload, timeout)
        response.process_state(status)
        return response

    def timeboard_set_current_step(self, step_index, timeout):
        """
        Set current a steps
        :param step_index: the index of the step
        :type step_index: int
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
        payload = json.dumps({
            'id': str(uuid.uuid4()),
            'timestamp': time.time(),
            'args': {
                'index': step_index
            }
        })
        status, result = self._execute_request("currentStep", payload, timeout)
        response.process_state(status)
        return response

    def timeboard_set_timer_state(self, activate, timeout):
        """
        Sets local manager timer state so that timeboard actually counts the passing of time
        :param activate: target state of the timer
        :type activate: bool
        :param timeout: maximum time to wait for a reaction from the local manager
        :type timeout: float
        """
        response = LTServiceResponse()
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
        response.process_state(status)
        return response

    #######################################
    # HRI MANAGEMENT METHODS
    ######################################

    def __naoqi_callback(self, payload):
        rospy.loginfo("{class_name}: Naoqi callback called. Payload: {payload}".format(
            class_name=self.__class__.__name__, payload=payload))
        payload_dict = json.loads(payload)
        lock = threading.Lock()
        lock.acquire()
        self._results_buffer[payload_dict["id"]] = payload_dict
        lock.release()

    def __is_result_received(self, goal_id):
        lock = threading.Lock()
        lock.acquire()
        result_received = (goal_id in self._results_buffer)
        lock.release()
        return result_received

    @staticmethod
    def __get_almemory_address(prefix, action_name, topic_type):
        return "{0}/{1}/{2}".format(prefix, action_name, topic_type)

    @staticmethod
    def __is_timeout_over(start_time, timeout):
        if timeout <= 0.0:
            return False
        return time.time() - start_time > timeout

    def __execute_request(self, action, payload, timeout):
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
            result_channel_subscriber.signal.connect(self.__naoqi_callback)
        except Exception as e:
            rospy.logerr("{class_name}: Could not initialize subscriber to result: {error}. ABORT !".format(
                class_name=self.__class__, error=e))
            return GoalStatus.ABORTED, None

        # Send goal on goal channel if channel as soon as channel is free
        try:
            goal_on_channel = json.loads(self._memory.getData(goal_channel))
            while goal_on_channel:
                if self.__is_timeout_over(start_time, timeout):
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
            while not self.__is_result_received(goal_id) and not rospy.is_shutdown():
                if self.__is_timeout_over(start_time, timeout):
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