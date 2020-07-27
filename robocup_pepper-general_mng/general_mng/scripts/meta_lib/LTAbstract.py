__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from actionlib_msgs.msg import GoalStatus


class LTAbstract:
    BUS = "BUS"
    ACTION = "ACTION"
    SERVICE = "SERVICE"

    #ACTION_WAIT_TIMEOUT = 20.0
    #SERVICE_WAIT_TIMEOUT = 10.0

    ACTION_WAIT_TIMEOUT = 1.0
    SERVICE_WAIT_TIMEOUT = 1.0
    REMOTE_DEVICE_WAIT_TIMEOUT = 1.0

    configurationReady = False

    @abstractmethod
    def reset(self):
        pass

    def action_status_to_string(self, action_status_int):
        if action_status_int == GoalStatus.PENDING:
            return "PENDING"
        elif action_status_int == GoalStatus.ACTIVE:
            return "ACTIVE"
        elif action_status_int == GoalStatus.PREEMPTED:
            return "PREEMPTED"
        elif action_status_int == GoalStatus.SUCCEEDED:
            return "SUCCEEDED"
        elif action_status_int == GoalStatus.ABORTED:
            return "ABORTED"
        elif action_status_int == GoalStatus.REJECTED:
            return "REJECTED"
        elif action_status_int == GoalStatus.PREEMPTING:
            return "PREEMPTING"
        elif action_status_int == GoalStatus.RECALLING:
            return "RECALLING"
        elif action_status_int == GoalStatus.RECALLED:
            return "RECALLED"
        elif action_status_int == GoalStatus.LOST:
            return "LOST"
