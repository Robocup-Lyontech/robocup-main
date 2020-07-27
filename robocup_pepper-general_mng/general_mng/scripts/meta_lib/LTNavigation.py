__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse
from actionlib_msgs.msg import GoalStatus

import actionlib
import rospy

from navigation_manager.msg import NavMngGoal, NavMngAction


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
class LTNavigation(LTAbstract):

    _enableNavAction = True

    def __init__(self):
        """
        Initializes the LTNavigation API. It will deal with every function related to navigation.
        """
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        """
        Loads the configuration needed to use correctly every navigation function.
        """
        rospy.loginfo("{class_name}: Connecting to navigation_manager action server ... ".format(class_name=self.__class__.__name__))

        self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
        finished1 = self._actionNavMng_server.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))

        if finished1:
            rospy.loginfo("{class_name}: navigation_manager Connected".format(class_name=self.__class__.__name__))
        else:
            rospy.logwarn("{class_name}: Unable to connect to navigation_manager action server".format(class_name=self.__class__.__name__))

    def reset(self):
        """
        Reloads the configuration needed to use correctly every navigation function.
        """
        self.configure_intern()

    def cancel_current_goals(self):
        self._actionNavMng_server.cancel_all_goals()

    #######################################
    # NAVIGATION API
    ######################################
    def send_nav_order(self, action, mode, it_point, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a navigation order with a specific action and a specific mode to reach an interest point in the map.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param action: navigation action
        :type action: string
        :param mode: navigation strategy
        :type mode: string
        :param it_point: interest point to reach
        :type it_point: geometry_msgs/Point
        :param timeout: maximum time to reach the interest point
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_order_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, mode, it_point, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_order to it:[%s], mode:[%s], action[%s]" % (
                it_point, mode, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes send_nav_order to it:[%s], mode:[%s], action[%s]" % (
                    it_point, mode, action)
                response.payload = feedback
                return response
        return response

    def send_nav_rotation_order(self,action, rotation_angle, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a navigation order with a specific action to rotate.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param action: navigation action
        :type action: string
        :param rotation_angle: rotation angle
        :type rotation_angle: float
        :param timeout: maximum time to execute the rotation
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_rotation_order,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, rotation_angle, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_rotation_order to rotation_angle:[%s], action[%s]" % (
                rotation_angle, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes send_nav_rotation_order to rotation_angle:[%s], action[%s]" % (
                    rotation_angle, action)
                response.payload = feedback
                return response
        return response

    def send_nav_order_to_pt(self, action, mode, x, y, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a navigation order with a specific action and a specific mode to reach a point in the map with specified coordinates.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param action: navigation action
        :type action: string
        :param mode: navigation strategy
        :type mode: string
        :param x: Coordinate X of the point to reach
        :type x: float
        :param y: Coordinate Y of the point to reach
        :type y: float
        :param timeout: maximum time to reach the interest point
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_order_to_pt_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, mode, x, y, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_order to it:[%s,%s], mode:[%s], action[%s]" % (
                    x, y, mode, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success send_nav_order to it:[%s,%s], mode:[%s], action[%s]" % (
                    x, y, mode, action)
                response.payload = feedback
                return response
        return response

    #######################################
    # NAVIGATION ACTION
    ######################################

    def __send_nav_order_action(self, action, mode, itP, timeout):
        """
        Action client which will send a navigation order with a specific action and a specific mode to reach an interest point in the map.
        Returns a GoalStatus and an action result

        :param action: navigation action
        :type action: string
        :param mode: navigation strategy
        :type mode: string
        :param itP: interest point to reach
        :type itP: geometry_msgs/Point
        :param timeout: maximum time to reach the interest point
        :type timeout: float
        """
        try:

            goal = NavMngGoal()
            goal.action = action
            goal.itP = itP
            goal.navstrategy = mode
            rospy.loginfo("{class_name}: ### NAV ACTION PENDING : %s".format(class_name=self.__class__.__name__), str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("{class_name}: ###### NAV ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            else:
                rospy.loginfo("{class_name}: ###### NAV ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("{class_name}: ###### NAV ACTION FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED

    def __send_nav_rotation_order(self, action, rotation_angle, timeout):
        """
        Action client which will send a navigation order with a specific action to rotate.
        Returns a GoalStatus and an action result.

        :param action: navigation action
        :type action: string
        :param rotation_angle: rotation angle
        :type rotation_angle: float
        :param timeout: maximum time to execute the rotation
        :type timeout: float
        """
        try:

            goal = NavMngGoal()
            goal.action = action
            goal.rotation_angle = rotation_angle
            rospy.loginfo("{class_name}: ### NAV ROTATION ACTION PENDING : %s".format(class_name=self.__class__.__name__), str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("{class_name}: ###### NAV ROTATION ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            else:
                rospy.loginfo("{class_name}: ###### NAV ROTATION ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("{class_name}: ###### NAV ROTATION ACTION FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED


    def __send_nav_order_to_pt_action(self, action, mode, x, y, timeout):
        """
        Action Client which will send a navigation order with a specific action and a specific mode to reach a point in the map with specified coordinates.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param action: navigation action
        :type action: string
        :param mode: navigation strategy
        :type mode: string
        :param x: Coordinate X of the point to reach
        :type x: float
        :param y: Coordinate Y of the point to reach
        :type y: float
        :param timeout: maximum time to reach the interest point
        :type timeout: float
        """
        try:
            goal = NavMngGoal()
            goal.action = action
            goal.itP = ''
            goal.itP_point.x = x
            goal.itP_point.y = y
            goal.navstrategy = mode
            rospy.loginfo("{class_name}: ### NAV ACTION PENDING : %s".format(class_name=self.__class__.__name__), str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("{class_name}: ###### NAV ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            else:
                rospy.loginfo("{class_name}: ###### NAV ACTION END , State: %s".format(class_name=self.__class__.__name__), self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("{class_name}: ###### NAV ACTION FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED
