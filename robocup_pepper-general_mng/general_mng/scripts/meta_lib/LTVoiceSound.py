__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse

import actionlib
import rospy

from tts_hri.msg import TtsHriGoal, TtsHriAction
from dialogue_hri_actions.msg import DialogueSendSignalAction, DialogueSendSignalGoal, AddInMemoryAction, \
    AddInMemoryGoal

from actionlib_msgs.msg import GoalStatus


class LTVoiceSound(LTAbstract):

    _enableTtsAction=True
    _enableDialogueAction=True
    _enableAddInMemoryAction=True

    def __init__(self):
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        rospy.loginfo("Connecting to tts_hri action server ... ")

        self._actionTtsHri_server = actionlib.SimpleActionClient('tts_hri', TtsHriAction)
        finished2 = self._actionTtsHri_server.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
        if finished2:
            rospy.loginfo("tts_hri Connected")
        else:
            rospy.logwarn("Unable to connect to tts_hri action server")

        self._actionDialogueHri_server = actionlib.SimpleActionClient('dialogue_hri_signal', DialogueSendSignalAction)
        finished3 = self._actionDialogueHri_server.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
        if finished3:
            rospy.loginfo("dialogue_hri Connected")
        else:
            rospy.logwarn("Unable to connect to dialogue_hri action server")

        if self._enableAddInMemoryAction:
            self._actionAddInMemoryHri_server = actionlib.SimpleActionClient('add_in_memory_action', AddInMemoryAction)
            finished4 = self._actionAddInMemoryHri_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished4:
                rospy.loginfo("AddInMemoryHri Connected")
            else:
                rospy.logwarn("Unable to connect to AddInMemoryHri action server")

    def reset(self):
        self.configure_intern()

    #######################################
    # VOICE API
    ######################################
    def send_tts_order(self, action, txt, mode, lang, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()
        # troncate String for log purpose
        txt_to_log = (txt[:75] + '..') if len(txt) > 75 else txt

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_tts_order_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for send_tts_order" % (service_mode)
            return response
        else:
            feedback = fct(action, txt, mode, lang, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_tts_order to txt:[%s], mode:[%s], action[%s], lang[%s]" % (
                txt_to_log, mode, action, lang)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success send_tts_order to txt:[%s], mode:[%s], action[%s], lang[%s]" % (
                txt_to_log, mode, action, lang)
                response.payload = feedback
                return response
        return response

    def send_dialogue(self, signal_to_emit, signal_to_wait, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_dialogue_order_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for send_dialogue" % (service_mode)
            return response
        else:
            feedback, result = fct(signal_to_emit, signal_to_wait, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_dialogue to signal_to_emit:[%s], signal_to_wait:[%s]" % (
                    signal_to_emit, signal_to_wait)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success send_dialogue to signal_to_emit:[%s], signal_to_wait:[%s]" % (
                    signal_to_emit, signal_to_wait)
                response.payload = result
                return response
        return response

    def add_in_pepper_memory(self, memory_location, json_payload, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__add_in_pepper_memory,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for add_in_pepper_memory" % (service_mode)
            return response
        else:
            feedback, result = fct(memory_location, json_payload, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during add_in_pepper_memory to memory_location:[%s], json_payload:[%s]" % (
                    memory_location, json_payload)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success add_in_pepper_memory to memory_location:[%s], json_payload:[%s]" % (
                    memory_location, json_payload)
                response.payload = result
                return response
        return response

    #######################################
    # VOICE ACTION
    ######################################

    def __send_tts_order_action(self, action, txt, mode, lang, timeout):
        try:
            goalTts = TtsHriGoal()
            goalTts.action = action
            goalTts.txt = txt
            goalTts.mode = mode
            goalTts.lang = lang
            rospy.loginfo("### TTS ACTION PENDING : %s", str(goalTts).replace('\n', ', '))
            self._actionTtsHri_server.send_goal(goalTts)
            self._actionTtsHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionTtsHri_server.get_state()
            rospy.loginfo("###### TTS ACTION END , State: %s", str(state))
            return state
        except Exception as e:
            rospy.logwarn("###### TTS ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED

    def __send_dialogue_order_action(self, signal_to_emit, signal_to_wait, timeout):
        try:
            goalDialogue = DialogueSendSignalGoal()
            # signal send to naoqi
            goalDialogue.signal_to_emit = signal_to_emit
            # signal on wich action server will wait answer
            goalDialogue.signal_to_wait = signal_to_wait
            rospy.loginfo("### DIALOGUE ACTION PENDING : %s", str(goalDialogue).replace('\n', ', '))
            # send the current goal to the action server
            self._actionDialogueHri_server.send_goal(goalDialogue)
            # wait action server result
            finished_before_timeout = self._actionDialogueHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionDialogueHri_server.get_state()
            result = self._actionDialogueHri_server.get_result()
            rospy.loginfo("###### DIALOGUE ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionDialogueHri_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### TTS ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED, None

    def __add_in_pepper_memory(self, memory_location, json_payload, timeout):
        goalAddInMemory = AddInMemoryGoal()
        goalAddInMemory.memory_location = memory_location
        goalAddInMemory.payload = json_payload
        rospy.loginfo("### ADD IN MEMORY ACTION PENDING : %s", str(goalAddInMemory).replace('\n', ', '))

        # send the current goal to the action server
        self._actionAddInMemoryHri_server.send_goal(goalAddInMemory)
        # wait action server result
        finished_before_timeout = self._actionAddInMemoryHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state = self._actionAddInMemoryHri_server.get_state()
        result = self._actionAddInMemoryHri_server.get_result()
        rospy.loginfo("###### ADD IN MEMORY ACTION END , State: %s", str(state))
        # if timeout cancel all goals on the action server
        if finished_before_timeout:
            self._actionAddInMemoryHri_server.cancel_all_goals()
        # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
        return state, result
