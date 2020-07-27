from actionlib_msgs.msg import GoalStatus


class LTServiceResponse:
    MAX_MSG_LENGTH = 150
    SUCCESS_STATUS = "SUCCESS"
    PENDING_STATUS = "PENDING"
    FAILURE_STATUS = "FAILURE"
    CANCELED_STATUS = "CANCELED"

    status = ""
    msg = ""
    payload = {}

    def __init__(self, current_status="FAILURE", current_msg="", current_payload={}):
        self.status = current_status
        self.msg = current_msg
        self.payload = current_payload

    def __str__(self):
        msg_to_log="[GM_SERVICE]:--->[%s]:  %s " % (self.status, self.msg)
        # troncate String for log purpose
        txt_to_log = (msg_to_log[:self.MAX_MSG_LENGTH] + '..') if len(msg_to_log) > self.MAX_MSG_LENGTH else msg_to_log
        return txt_to_log

    def process_state(self, goalStatus):
        if (goalStatus == GoalStatus.ABORTED or goalStatus == GoalStatus.LOST or goalStatus == GoalStatus.REJECTED):
            self.status = self.FAILURE_STATUS
            return
        elif (goalStatus == GoalStatus.PENDING or goalStatus == GoalStatus.ACTIVE):
            self.status = self.PENDING_STATUS
            return
        elif (
                goalStatus == GoalStatus.PREEMPTED or goalStatus == GoalStatus.PREEMPTING or goalStatus == GoalStatus.RECALLED or goalStatus == GoalStatus.RECALLING):
            self.status = self.CANCELED_STATUS
            return
        elif (goalStatus == GoalStatus.SUCCEEDED):
            self.status = self.SUCCESS_STATUS
            return
        else:
            # if status is unknown return failure state
            self.status = self.FAILURE_STATUS
            return
