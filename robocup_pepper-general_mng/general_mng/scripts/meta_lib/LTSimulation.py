__author__ = 'Thomas CURE'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse
from actionlib_msgs.msg import GoalStatus

import rospy
import actionlib
from pmb2_apps.msg import SdfInGazeboAction, SdfInGazeboGoal
from geometry_msgs.msg import Pose
import tf
import os


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
class LTSimulation(LTAbstract):

    def __init__(self):
        """
        Initializes the LTSimulation API. It will deal with every function related to simulation environment.
        """

        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True
        rospy.loginfo("{class_name}: LTSimulation initialized".format(class_name=self.__class__.__name__))
    
    
    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        """
        Loads the configuration needed to use correctly every simulation function.
        """

        self.spawner_gazebo_client = actionlib.SimpleActionClient("sdf_in_gazebo_action",SdfInGazeboAction)
        server_is_up = self.spawner_gazebo_client.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
        if server_is_up:
            rospy.loginfo("{class_name}: Gazebo SDF spawner connected".format(class_name=self.__class__.__name__))
        else:
            rospy.logwarn("{class_name}: Gazebo SDF spawner disconnected".format(class_name=self.__class__.__name__))


        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.models_dir = os.path.join(self.current_dir,"../../../../../Palbator_simulation/pmb2_simulation/pmb2_gazebo/models")
    def reset(self):
        """
        Reloads the configuration needed to use correctly every simulation function.
        """
        self.configure_intern()

    #######################################
    # SIMULATION API
    ######################################

    def reset_guests_for_receptionist(self,service_mode=LTAbstract.ACTION):
        """ 
        Will send a request to the gazebo sdf spawner to reset the spawned guests.
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__reset_guests_for_receptionist,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for guest_spawner_for_receptionist" % (service_mode)
            return response
        else:
            feedback, result = fct()
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during guest_spawner_for_receptionist"
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes guest_spawner_for_receptionist" 
                response.result = result
                return response
        return response

    def guest_spawner_for_receptionist(self,mode_spawner,service_mode=LTAbstract.ACTION):
        """ 
        Will send a request to the gazebo sdf spawner to spawn a guest in a specified position
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__guest_spawner_for_receptionist,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for guest_spawner_for_receptionist" % (service_mode)
            return response
        else:
            feedback, result = fct(mode_spawner)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during guest_spawner_for_receptionist"
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes guest_spawner_for_receptionist" 
                response.result = result
                return response
        return response

    #######################################
    # MOTION ACTION
    ######################################

    def __reset_guests_for_receptionist(self):
        goal = SdfInGazeboGoal()
        goal.action = "delete"
        goal.model_name = "Guest_1"
        self.spawner_gazebo_client.send_goal(goal)
        self.spawner_gazebo_client.wait_for_result()
        response = self.spawner_gazebo_client.get_result()
        rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))

        goal = SdfInGazeboGoal()
        goal.action = "delete"
        goal.model_name = "Guest_2"
        self.spawner_gazebo_client.send_goal(goal)
        self.spawner_gazebo_client.wait_for_result()
        response = self.spawner_gazebo_client.get_result()
        rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))

        return GoalStatus.SUCCEEDED, "OK"


    def __guest_spawner_for_receptionist(self,mode_spawner):
        try:
            if mode_spawner == "G1_entrance":
                goal = SdfInGazeboGoal()
                goal.action = "spawn"
                goal.model_name = "Guest_1"
                goal.path_to_sdf = os.path.join(self.models_dir,"citizen_extras_female_03/model.sdf")
                # "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_03/model.sdf"
                goal.model_pose = Pose()
                goal.model_pose.position.x = -0.2
                goal.model_pose.position.y = -5.5
                q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -3.088)
                goal.model_pose.orientation.x = q[0]
                goal.model_pose.orientation.y = q[1]
                goal.model_pose.orientation.z = q[2]
                goal.model_pose.orientation.w = q[3]

                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))
                return GoalStatus.SUCCEEDED, response
            
            elif mode_spawner == "G1_before_present":
                goal = SdfInGazeboGoal()
                goal.action = "delete"
                goal.model_name = "Guest_1"
                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))

                goal = SdfInGazeboGoal()
                goal.action = "spawn"
                goal.model_name = "Guest_1"
                goal.path_to_sdf = os.path.join(self.models_dir,"citizen_extras_female_03/model.sdf")
                goal.model_pose = Pose()
                goal.model_pose.position.x = 1.5
                goal.model_pose.position.y = 5.5
                q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -1.149950)
                goal.model_pose.orientation.x = q[0]
                goal.model_pose.orientation.y = q[1]
                goal.model_pose.orientation.z = q[2]
                goal.model_pose.orientation.w = q[3]

                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))
                return GoalStatus.SUCCEEDED, response

            elif mode_spawner == "G1_after_present":
                goal = SdfInGazeboGoal()
                goal.action = "delete"
                goal.model_name = "Guest_1"
                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))

                goal = SdfInGazeboGoal()
                goal.action = "spawn"
                goal.model_name = "Guest_1"
                goal.path_to_sdf = os.path.join(self.models_dir,"citizen_extras_female_03/model.sdf")
                goal.model_pose = Pose()
                goal.model_pose.position.x = -0.5
                goal.model_pose.position.y = 6.1
                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))
                return GoalStatus.SUCCEEDED, response
            
            elif mode_spawner == "G2_entrance":
                goal = SdfInGazeboGoal()
                goal.action = "spawn"
                goal.model_name = "Guest_2"
                goal.path_to_sdf = os.path.join(self.models_dir,"person_standing/model.sdf")
                goal.model_pose = Pose()
                goal.model_pose.position.x = -0.2
                goal.model_pose.position.y = -5.5
                q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -3.088)
                goal.model_pose.orientation.x = q[0]
                goal.model_pose.orientation.y = q[1]
                goal.model_pose.orientation.z = q[2]
                goal.model_pose.orientation.w = q[3]

                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))
                return GoalStatus.SUCCEEDED, response

            elif mode_spawner == "G2_before_present":
                goal = SdfInGazeboGoal()
                goal.action = "delete"
                goal.model_name = "Guest_2"
                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))

                goal = SdfInGazeboGoal()
                goal.action = "spawn"
                goal.model_name = "Guest_2"
                goal.path_to_sdf = os.path.join(self.models_dir,"person_standing/model.sdf")
                goal.model_pose = Pose()
                goal.model_pose.position.x = 1.5
                goal.model_pose.position.y = 5.5
                q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -1.149950)
                goal.model_pose.orientation.x = q[0]
                goal.model_pose.orientation.y = q[1]
                goal.model_pose.orientation.z = q[2]
                goal.model_pose.orientation.w = q[3]

                self.spawner_gazebo_client.send_goal(goal)
                self.spawner_gazebo_client.wait_for_result()
                response = self.spawner_gazebo_client.get_result()
                rospy.loginfo("{class_name}: RESPONSE FROM GAZEBO : %s".format(class_name=self.__class__.__name__),str(response))
                return GoalStatus.SUCCEEDED, response
            else:
                rospy.logwarn("{class_name}: Unknown mode for receptionist guests spawner".format(class_name=self.__class__.__name__))
                return GoalStatus.ABORTED, None

        except Exception as e:
            rospy.logerr("{class_name}: Action __guest_spawner_for_receptionist could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None