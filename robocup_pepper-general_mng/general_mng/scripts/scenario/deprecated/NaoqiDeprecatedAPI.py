####################################################################################################################
# Start of API between GeneralManager and HRI
####################################################################################################################
def _send_goal_and_wait(self, goal, timeout):
    self._actionRequestToLocalManager_server.send_goal(goal)
    self._actionRequestToLocalManager_server.wait_for_result(rospy.Duration.from_sec(timeout))
    return (self._actionRequestToLocalManager_server.get_state(),
            self._actionRequestToLocalManager_server.get_result())


def ask_name(self, speech, names, timeout):
    """
    Start the view 'askName'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param names: a list of names that can be chosen by operators
    :type names: list
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="askName", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech,
            'names': names
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


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
    goal = RequestToLocalManagerGoal(action="askDrink", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech,
            'drinks': drinks
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def ask_age(self, speech, timeout):
    """
    Start the view 'askAge'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="askAge", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def confirm(self, speech, timeout):
    """
    Start the view 'confirm'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="confirm", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def detail_drinks(self, timeout):
    raise NotImplementedError()


def present_person(self, timeout):
    raise NotImplementedError()


def go_to(self, speech, timeout):
    """
    Start the view 'goTo'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="goTo", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def seat_guest(self, speech, guest, timeout):
    """
    Start the view 'seatGuest'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param guest: name of the guest to seat
    :type guest: string
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="seatGuest", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech,
            'guest': guest
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def ask_to_follow(self, speech, timeout):
    """
    Start the view 'askToFollow'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="askToFollow", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


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
    goal = RequestToLocalManagerGoal(action="mainMenu", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech,
            'scenarios': scenarios
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


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
    goal = RequestToLocalManagerGoal(action="wait", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'time': waiting_time,
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def ask_open_door(self, speech, timeout):
    """
    Start the view 'askOpenDoor'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="askOpenDoor", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def call_human(self, speech, timeout):
    """
    Start the view 'callHuman'

    :param speech: the text that will be use by the Local Manager for tablet and vocal
    :type speech: dict
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="callHuman", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


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
    goal = RequestToLocalManagerGoal(action="showVideo", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'speech': speech,
            'video_url': video_url
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def find_available_drinks(self, timeout):
    raise NotImplementedError()


def open_door(self, timeout):
    raise NotImplementedError()


def find_who_wants_drinks(self, timeout):
    raise NotImplementedError()


def serve_drinks(self, timeout):
    raise NotImplementedError()


# Timeboard API

def timeboard_set_current_step(self, step_index, timeout):
    """
    Set current a steps

    :param step_index: the index of the step
    :type step_index: int
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="currentStep", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'index': step_index
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def timeboard_send_step_done(self, step_indexes, timeout):
    """
    Set done a list of steps

    :param step_indexes: A list of indexes of the steps
    :type step_indexes: list
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="stepDone", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'indexes': step_indexes
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


def timeboard_send_step_skipped(self, step_indexes, timeout):
    """
    Set skipped a list of steps

    :param step_indexes: A list of indexes of the steps
    :type step_indexes: list
    :param timeout: maximum time to wait for a reaction from the local manager
    :type timeout: float
    """
    goal = RequestToLocalManagerGoal(action="stepSkipped", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'indexes': step_indexes
        }
    }))
    return self._send_goal_and_wait(goal, timeout)


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
            step_list.append({'name': step['name'], 'eta': step['eta']})
            step_id_to_index[step[id]] = index
            index += 1

    goal = RequestToLocalManagerGoal(action="stepsList", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'scenarioName': scenario_name,
            'stepsList': step_list
        }
    }))
    return self._send_goal_and_wait(goal, timeout), step_id_to_index


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

    goal = RequestToLocalManagerGoal(action="toggleTimer", payload=json.dumps({
        'id': str(uuid.uuid4()),
        'timestamp': time.time(),
        'args': {
            'state': LOCAL_MANAGER_TIMER_ON_MSG if activate else LOCAL_MANAGER_TIMER_OFF_MSG
        }
    }))
    return self._send_goal_and_wait(goal, timeout)

# TODO Add Timer start function

####################################################################################################################
# End of API between GeneralManager and HRI
####################################################################################################################
