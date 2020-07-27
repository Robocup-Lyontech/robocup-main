__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse

import actionlib
import rospy

from ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgGoal
from ros_people_mng_actions.msg import LearnPeopleFromImgAction, LearnPeopleFromImgGoal
from ros_people_mng_actions.msg import GetPeopleNameFromImgAction, GetPeopleNameFromImgGoal
from ros_people_mng_srvs.srv import TakePictureService

from object_management.msg import ObjectDetectionAction, ObjectDetectionGoal
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import GoalStatus
import cv2
from rospy.exceptions import ROSException, ROSInterruptException
from std_srvs.srv import Trigger
from pepper_door_open_detector.srv import MinFrontValue
from dialogue_hri_srvs.srv import TakePicture

from tf_broadcaster.srv import GetObjectsInRoom, GetPeopleInRoom, ResetObjects
from tf import TransformListener
import math


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
class LTPerception(LTAbstract):

    OPEN_DOOR_MIN_DISTANCE = 0.8
    CHECK_DISTANCE_FREQUENCY = 10

    _enableObjectDetectionMngAction = True
    _enableLearnPeopleMetaAction = True
    _enableMultiplePeopleDetectionAction = True
    _enableGetPeopleNameAction = True
    _enableMinFrontValueService = True
    _enableResetPersonMetaInfoMapService = True
    _enableResetPersonMetaInfoMapService = True
    _enableTakePictureService = True

    _enableCheckForObjectsInRoom = True
    _enableTakePicturePalbator = True
    _enableCheckForPeopleInRoom = True
    _enableResetObjects = True



    def __init__(self):
        """
        Initializes the LTPerception API. It will deal with every function related to perception.
        """
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        """
        Loads the configuration needed to use correctly every perception function.
        """
        rospy.loginfo("{class_name}: Loading configuration for LTPerception ... ".format(class_name=self.__class__.__name__))

        if self._enableResetObjects:
            service_name = "reset_object_files"
            self.reset_objects_proxy = rospy.ServiceProxy(service_name, ResetObjects)
            try:
                rospy.wait_for_service(service_name,timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: reset objects server connected".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the reset objects service.".format(class_name=self.__class__.__name__))

        if self._enableTakePicturePalbator:
            service_name = "take_picture_service"
            self.take_picture_palbator_proxy = rospy.ServiceProxy(service_name,TakePictureService)
            try:
                rospy.wait_for_service(service_name,timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: take_picture_service server connected".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the take_picture_service service.".format(class_name=self.__class__.__name__))

        if self._enableCheckForObjectsInRoom:
            service_name = 'get_objects_list'
            self.get_object_in_room_proxy = rospy.ServiceProxy(service_name,GetObjectsInRoom)
            try:
                rospy.wait_for_service(service_name,timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: get_objects_in_room server connected".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the get_objects_in_room service.".format(class_name=self.__class__.__name__))

        if self._enableCheckForPeopleInRoom:
            service_name = 'get_people_list_in_room'
            self.get_people_in_room_proxy = rospy.ServiceProxy(service_name,GetPeopleInRoom)
            try:
                rospy.wait_for_service(service_name,timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: get_people_in_room server connected".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the get_people_in_room service.".format(class_name=self.__class__.__name__))
        
        if self._enableObjectDetectionMngAction:
            self._actionObjectDetectionMng_server = actionlib.SimpleActionClient('object_detection_action',
                                                                                 ObjectDetectionAction)
            finished5 = self._actionObjectDetectionMng_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished5:
                rospy.loginfo("{class_name}: ObjectDetectionMng Connected".format(class_name=self.__class__.__name__))
            else:
                rospy.logwarn("{class_name}: Unable to connect to ObjectDetectionMng action server".format(class_name=self.__class__.__name__))

        if self._enableLearnPeopleMetaAction:
            self._actionLearnPeopleMeta_server = actionlib.SimpleActionClient('learn_people_meta_action',
                                                                              LearnPeopleFromImgAction)
            finished7 = self._actionLearnPeopleMeta_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished7:
                rospy.loginfo("{class_name}: learnPeopleMeta action server Connected".format(class_name=self.__class__.__name__))
            else:
                rospy.logwarn("{class_name}: Unable to connect to learnPeopleMeta action server".format(class_name=self.__class__.__name__))

        if self._enableMultiplePeopleDetectionAction:
            self._actionMultiplePeopleDetection_server = actionlib.SimpleActionClient('detect_people_meta_action',
                                                                                      ProcessPeopleFromImgAction)
            finished8 = self._actionMultiplePeopleDetection_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished8:
                rospy.loginfo("{class_name}: MultiplePeopleDetection Connected".format(class_name=self.__class__.__name__))
            else:
                rospy.logwarn("{class_name}: Unable to connect to MultiplePeopleDetection action server".format(class_name=self.__class__.__name__))

        if self._enableGetPeopleNameAction:
            self._actionGetPeopleName_server = actionlib.SimpleActionClient('get_people_name_action',
                                                                            GetPeopleNameFromImgAction)
            finished9 = self._actionGetPeopleName_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished9:
                rospy.loginfo("{class_name}: GetPeopleName Connected".format(class_name=self.__class__.__name__))
            else:
                rospy.logwarn("{class_name}: Unable to connect to GetPeopleName action server".format(class_name=self.__class__.__name__))

        if self._enableResetPersonMetaInfoMapService:
            rospy.loginfo("{class_name}: Connecting to the reset_people_meta_info_map_srv service...".format(class_name=self.__class__.__name__))
            self._resetPeopleMetaInfoMapSP = rospy.ServiceProxy('reset_people_meta_info_map_srv', Trigger)
            try:
                reset_people_meta_info_map_srv_is_up = rospy.wait_for_service('reset_people_meta_info_map_srv',
                                                                              timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: Connected to the reset_people_meta_info_map_srv service.".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the reset_people_meta_info_map_srv service.".format(class_name=self.__class__.__name__))

        if self._enableMinFrontValueService:
            rospy.loginfo("{class_name}: Connecting to the min_front_value_srv service...".format(class_name=self.__class__.__name__))
            self._minFrontValueSP = rospy.ServiceProxy('min_front_value_srv', MinFrontValue)
            try:
                min_front_value_srv_is_up = rospy.wait_for_service('min_front_value_srv', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: Connected to the min_front_value_srv service.".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the min_front_value_srv service.".format(class_name=self.__class__.__name__))

        # Connect to take_picture_service service
        if self._enableTakePictureService:
            rospy.loginfo("{class_name}: Connecting to the take_picture_service service...".format(class_name=self.__class__.__name__))
            self._takePictureSP = rospy.ServiceProxy('take_picture_service', TakePicture)
            try:
                take_picture_srv_is_up = rospy.wait_for_service('take_picture_service', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: Connected to the take_picture_service service.".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the take_picture_service service.".format(class_name=self.__class__.__name__))

        self._bridge = CvBridge()

    def reset(self):
        """
        Reloads the configuration needed to use correctly every perception function.
        """
        self.configure_intern()

    #######################################
    # PERCEPTION API
    ######################################

    # def get_object_in_front_robot(self, labels, move_head, timeout, service_mode=LTAbstract.ACTION):
    #     response = LTServiceResponse()
    #
    #     # Check different service mode
    #     switcher = {
    #         LTAbstract.ACTION: self.__get_object_in_front_robot,
    #         LTAbstract.BUS: None,
    #         LTAbstract.SERVICE: None
    #     }
    #
    #     fct = switcher[service_mode]
    #
    #     # if service mode not available return an Failure
    #     if fct is None:
    #         response.status = LTServiceResponse.FAILURE_STATUS
    #         response.msg = " is not available for get_object_in_front_robot" % (service_mode)
    #         return response
    #     else:
    #         feedback, result = fct(labels, move_head, timeout)
    #         response.process_state(feedback)
    #
    #         if response.status == LTServiceResponse.FAILURE_STATUS:
    #             response.msg = " Failure during get_object_in_front_robot to labels:[%s], move_head:[%s]" % (
    #                 labels, move_head)
    #             return response
    #         else:
    #             # FIXME to be completed with all ACTION status in GoalStatus
    #             response.msg = " Operation success get_object_in_front_robot to to labels:[%s], move_head:[%s]" % (
    #                 labels, move_head)
    #             response.payload = result
    #             return response
    #     return response

    def reset_objects_in_map_manager(self,all_objects, object_label, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__reset_objects_in_map_manager,
        }
        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for reset_objects_in_map_manager" % (service_mode)
            return response
        else:
            feedback, result = fct(all_objects,object_label)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during reset_objects_in_map_manager "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success reset_objects_in_map_manager "
                response.payload = result
                return response
        return response

    def get_people_in_room(self,room,service_mode=LTAbstract.SERVICE):
        """
        Will send a perception order with a specific room in order to get the list of people in that room. 
        Returns a response containing the result, the status and the feedback of the executed action.

        :param room: name of the room in which the system will look for people
        :type room: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__get_people_in_room,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_in_room" % (service_mode)
            return response
        else:
            feedback, result = fct(room)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_in_room "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_in_room "
                response.payload = result
                return response
        return response

    def get_object_in_room(self,room,service_mode=LTAbstract.SERVICE):
        """
        Will send a perception order with a specific room in order to get the list of objects from that room. 
        Returns a response containing the result, the status and the feedback of the executed action.

        :param room: name of the room in which the system will look for objects
        :type room: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__get_object_in_room,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_object_in_room" % (service_mode)
            return response
        else:
            feedback, result = fct(room)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_object_in_room "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_object_in_room "
                response.payload = result
                return response
        return response

    def detect_objects_with_look_around(self, labels, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a list of object labels in order to detect only those objects.
        During this action, the robot will look around moving its head (for a robot like Pepper for instance).
        Returns a response containing the result, the status and the feedback of the executed action.

        :param labels: List of the object labels the system can detect
        :type labels: list
        :param timeout: maximum time to execute the detection
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_objects_with_look_around,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_objects_with_look_around" % (service_mode)
            return response
        else:
            feedback, result = fct(labels, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_objects_with_look_around to labels:[%s]" % (
                    labels)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_objects_with_look_around to to labels:[%s]s" % (
                    labels)
                response.payload = result
                return response
        return response

    def detect_objects_with_given_sight_from_img_topic(self, labels, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a list of object labels in order to detect only those objects.
        The system will take images from an image topic.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param labels: List of the object labels the system can detect
        :type labels: list
        :param timeout: maximum time to execute the detection
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_objects_with_given_sight_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_objects_with_given_sight_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(labels, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_objects_with_given_sight_from_img_topic to labels:[%s]" % (
                    labels)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_objects_with_given_sight_from_img_topic to to labels:[%s]s" % (
                    labels)
                response.payload = result
                return response
        return response

    def detect_objects_with_given_sight_from_img_path(self, labels, path, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a list of object labels in order to detect only those objects and a path to an image.
        The system will look for the objects in the specified image.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param labels: List of the object labels the system can detect
        :type labels: list
        :param path: path to an image which will be analyzed by the detection tool.
        :type path: string
        :param timeout: maximum time to execute the detection
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_objects_with_given_sight_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_objects_with_given_sight_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(labels, path, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_objects_with_given_sight_from_img_topic to labels:[%s] path:[%s]" % (
                    labels,path)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_objects_with_given_sight_from_img_topic to labels:[%s] path:[%s]" % (
                    labels,path)
                response.payload = result
                return response
        return response

    def take_picture_and_save_it(self, path, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a path to an image in order to take a picture and save it at the path location.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param path: path to save the picture
        :type path: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__take_picture_and_save_it,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for take_picture_and_save_it" % (service_mode)
            return response
        else:
            feedback, result = fct( path)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during take_picture_and_save_it to path:[%s]" % (
                    path)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success take_picture_and_save_it to path:[%s]" % (
                    path)
                response.payload = result
                return response
        return response

    def take_picture_and_save_it_Palbator(self, path, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a path to an image in order to take a picture and save it at the path location.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param path: path to save the picture
        :type path: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__take_picture_and_save_it_Palbator,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for take_picture_and_save_it_Palbator" % (service_mode)
            return response
        else:
            feedback, result = fct( path)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during take_picture_and_save_it_Palbator to path:[%s]" % (
                    path)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success take_picture_and_save_it_Palbator to path:[%s]" % (
                    path)
                response.payload = result
                return response
        return response

    def learn_people_meta_from_img_topic(self, name, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a specified name. The system will look for a face on its image topic and will associate the detected face to the name.
        When this face will be detected later, the system will be able to say that this face belongs to the specified name.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param name: name to be learnt by the system associating it wih the face detection
        :type name: string
        :param timeout: maximum time to execute the face detection and learning procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__learn_people_meta_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(name, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during learn_people_meta_from_img_topic to name:[%s]" % (
                    name)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success learn_people_meta_from_img_topic to name:[%s]" % (
                    name)
                response.payload = result
                return response
        return response

    def learn_people_meta_from_img_path(self, img_path, name, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a specified name and an image path. The system will look for a face on the specified image
        and will associate the detected face to the name.
        When this face will be detected later, the system will be able to say that this face belongs to the specified name.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param img_path: path to find the image with the people to learn
        :type img_path: string
        :param name: name to be learnt by the system associating it wih the face detection
        :type name: string
        :param timeout: maximum time to execute the face detection and learning procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__learn_people_meta_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(img_path, name, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during _learn_people_meta_from_img_path to img_path:[%s] name:[%s]" % (
                    img_path, name)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success _learn_people_meta_from_img_path to img_path:[%s] name:[%s]" % (
                    img_path, name)
                response.payload = result
                return response
        return response

    def learn_people_meta(self, goalLearnPeople, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order with a specified goal. The system will associate the people attributes in the goal image it will receive with the name specified in the goal.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param goalLearnPeople: action goal containing a name and an img
        :type goalLearnPeople: ros_people_mng_actions.msg/LearnPeopleFromImgGoal 
        :param timeout: maximum time to execute the face detection and learning procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__learn_people_meta,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta" % (service_mode)
            return response
        else:
            feedback, result = fct(goalLearnPeople, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during learn_people_meta to goalLearnPeople:[%s] " % (
                    goalLearnPeople)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success learn_people_meta to goalLearnPeople:[%s] " % (
                    goalLearnPeople)
                response.payload = result
                return response
        return response

    def detect_meta_people_from_img_topic(self, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the system image topic.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_meta_people_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people_from_img_topic "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people_from_img_topic "
                response.payload = result
                return response
        return response

    def detect_meta_people_from_img_path(self, img_path, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the image in the specified path.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param img_path: image path where the system will find the image to analyze.
        :type img_path: string
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_meta_people_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(img_path, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people_from_img_path to img_path:[%s] " % (
                    img_path)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people_from_img_path to img_path:[%s]" % (
                    img_path)
                response.payload = result
                return response
        return response

    def detect_meta_people(self, goalMetaPeople, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the image in the specified goal.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param goalMetaPeople: Action goal which contains the image to analyze
        :type goalMetaPeople: ros_people_mng_actions.msg/ProcessPeopleFromImgGoal
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__detect_meta_people,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people" % (service_mode)
            return response
        else:
            feedback, result = fct(goalMetaPeople, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people to goalMetaPeople:[%s] " % (
                    goalMetaPeople)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people to goalMetaPeople:[%s]" % (
                    goalMetaPeople)
                response.payload = result
                return response
        return response

    def get_people_name_from_img_topic(self, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the image topic and get the people names.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__get_people_name_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name_from_img_topic "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name_from_img_topic "
                response.payload = result
                return response
        return response

    def get_people_name(self, goalPeopleName, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the image specified in the goal and get the people names.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param goalPeopleName: ActionGoal containing the image to analyze
        :type goalPeopleName: ros_people_mng_actions.msg/GetPeopleNameFromImgGoal
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__get_people_name,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name" % (service_mode)
            return response
        else:
            feedback, result = fct(goalPeopleName, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name "
                response.payload = result
                return response
        return response

    def get_people_name_from_img_path(self, img_path, timeout, service_mode=LTAbstract.ACTION):
        """
        Will send a perception order to start a people detection on the image located in the image path and get the people names.
        Returns a response containing the result, the status and the feedback of the executed action.

        :param img_path: path of the image to analyze
        :type img_path: string
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__get_people_name_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(img_path, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name_from_img_path to goalPeopleName:[%s] " % (
                    goalPeopleName)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name_from_img_path to goalPeopleName:[%s]" % (
                    goalPeopleName)
                response.payload = result
                return response
        return response

    def reset_people_meta_info_map(self, service_mode=LTAbstract.SERVICE):
        """
        Will send a perception order to reset the people infos the system had learnt so far.
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__reset_people_meta_info_map,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for reset_people_meta_info_map" % (service_mode)
            return response
        else:
            feedback, result = fct()
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during reset_people_meta_info_map "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success reset_people_meta_info_map "
                response.payload = result
                return response
        return response

    def wait_for_door_to_open(self, check_freq=None, min_dist=None, service_mode=LTAbstract.SERVICE):
        """
        Will send a perception order to wait for the door to open.
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        if min_dist is None:
            min_dist = self.OPEN_DOOR_MIN_DISTANCE
        if check_freq is None:
            check_freq = self.CHECK_DISTANCE_FREQUENCY

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__wait_for_door_to_open,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for wait_for_door_to_open" % (service_mode)
            return response
        else:
            feedback, result = fct(check_freq, min_dist)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during wait_for_door_to_open check_freq:[%s], min_dist[%s] " % (
                check_freq, min_dist)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success wait_for_door_to_open check_freq:[%s], min_dist[%s] " % (
                check_freq, min_dist)
                response.payload = result
                return response
        return response

    #######################################
    # PERCEPTION ACTION
    ######################################

    def __reset_objects_in_map_manager(self,all_objects, object_label):
        try:
            self.reset_objects_proxy(all_objects,object_label)
            return GoalStatus.SUCCEEDED, None
        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service reset_object_in_map_manager could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service reset_object_in_map_manager could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
    
    def __get_people_in_room(self,room):
        """
        Service client which will send a perception order with a specific room in order to get the list of people in that room. 
        Returns a GoalStatus and a people list

        :param room: name of the room in which the system will look for people
        :type room: string
        """
        try:
            result = self.get_people_in_room_proxy(room)
            people_list = result.people_list

            return GoalStatus.SUCCEEDED, people_list

        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service get_people_in_room could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service get_people_in_room could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __get_object_in_room(self,room):
        """
        Service client which will send a perception order with a specific room in order to get the list of objects from that room. 
        Returns a GoalStatus and an objects list

        :param room: name of the room in which the system will look for objects
        :type room: string
        """
        try:
            result = self.get_object_in_room_proxy(room)
            objects_list = result.objects_list

            return GoalStatus.SUCCEEDED, objects_list

        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service get_object_in_room could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service get_object_in_room could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None



    def __detect_objects_with_look_around(self, labels, timeout):
        """
        Pepper will move its head to find the desired objects with Darknet and the image flow.
        """

        # Create the goal
        goalObjDetection = ObjectDetectionGoal()
        goalObjDetection.labels = labels
        goalObjDetection.moveHead = True
        # Outputs
        state, result = self.__detectObjects(goalObjDetection, timeout)

        return state, result

    def __detect_objects_with_given_sight_from_img_topic(self, labels, timeout):
        """
        Darknet will try to detect objects in the current image flow
        """
        # Create the goal
        goalObjDetection = ObjectDetectionGoal()
        goalObjDetection.labels = labels
        goalObjDetection.moveHead = False
        # Outputs
        state, result = self.__detectObjects(goalObjDetection, timeout)
        return state, result

    def __detect_objects_with_given_sight_from_img_path(self, labels, path, timeout):
        """
        Darknet will try to detect objects in a image file
        """
        # Create the goal
        goalObjDetection = ObjectDetectionGoal()
        goalObjDetection.labels = labels
        goalObjDetection.path = path
        goalObjDetection.moveHead = False
        # Outputs
        state, result = self.__detectObjects(goalObjDetection, timeout)
        return state, result

    def __detectObjects(self, goal, timeout):
        """
        Private function.
        Send an ObjectDetection goal to the related action server and return the results.
        """
        try:
            rospy.loginfo("{class_name}: ### OBJECT DETECTION MNG GET OBJECT ACTION PENDING : %s".format(class_name=self.__class__.__name__), str(goal).replace('\n', ', '))
            # send the current goal to the action server
            self._actionObjectDetectionMng_server.send_goal(goal)
            # wait action server result
            finished_before_timeout = self._actionObjectDetectionMng_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionObjectDetectionMng_server.get_state()
            result = self._actionObjectDetectionMng_server.get_result()
            rospy.loginfo("{class_name}: ###### OBJECT DETECTION MNG GET OBJECT ACTION END , State: %s".format(class_name=self.__class__.__name__), str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionObjectDetectionMng_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("{class_name}: ###### OBJECT DETECTION MNG ACTION FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED, None

##--------------------


    # def __get_object_in_front_robot(self, labels, move_head, timeout):
    #     try:
    #         goalObjDetection = ObjectDetectionGoal()
    #         goalObjDetection.labels = labels
    #         goalObjDetection.moveHead = move_head
    #
    #         rospy.loginfo("### OBJECT DETECTION MNG GET OBJECT ACTION PENDING : %s",
    #                       str(goalObjDetection).replace('\n', ', '))
    #
    #         # send the current goal to the action server
    #         self._actionObjectDetectionMng_server.send_goal(goalObjDetection)
    #         # wait action server result
    #         finished_before_timeout = self._actionObjectDetectionMng_server.wait_for_result(
    #             rospy.Duration.from_sec(timeout))
    #         state = self._actionObjectDetectionMng_server.get_state()
    #         result = self._actionObjectDetectionMng_server.get_result()
    #         rospy.loginfo("###### OBJECT DETECTION MNG GET OBJECT ACTION END , State: %s", str(state))
    #         # if timeout cancel all goals on the action server
    #         if finished_before_timeout:
    #             self._actionObjectDetectionMng_server.cancel_all_goals()
    #         # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
    #         return state, result
    #     except Exception as e:
    #         rospy.logwarn("###### OBJECT DETECTION MNG ACTION FAILURE , State: %s", str(e))
    #     return GoalStatus.ABORTED, None

    def __take_picture_and_save_it(self, path):
        """
        Take a picture with the naoqi request API and then save it in a dedicated file in png format.
        path must be the picture absolute path.
        """
        try:
            result = self._takePictureSP(path)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service take_picture_service could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service take_picture_service could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __take_picture_and_save_it_Palbator(self, path):
        """
        Take a picture and then save it in a dedicated file in png format.
        path must be the picture absolute path.
        """
        try:
            result = self.take_picture_palbator_proxy(path)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service take_picture_service_palbator could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service take_picture_service_palbator could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        

    def __learn_people_meta_from_img_topic(self, name, timeout):
        """ Appel de l'apprentissage des attributs d'une personne """
        goalLearnPeople = LearnPeopleFromImgGoal(name=name)
        rospy.logwarn("{class_name}: GOAL : %s".format(class_name=self.__class__.__name__),str(goalLearnPeople))

        state, result = self.__learn_people_meta(goalLearnPeople, timeout)
        return state, result

    def __learn_people_meta_from_img_path(self, img_path, name, timeout):
        """ Appel de l'apprentissage des attributs d'une personne """
        img_loaded = cv2.imread(img_path)
        msg_img = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
        goalLearnPeople = LearnPeopleFromImgGoal(name=name, img=msg_img)
        state, result = self.__learn_people_meta(goalLearnPeople, timeout)
        return state, result

    def __learn_people_meta(self, goalLearnPeople, timeout):
        """
        Action client which will send a perception order with a specified goal. 
        The system will associate the people attributes in the goal image it will receive with the name specified in the goal.
        Returns a GoalStatus and an action result

        :param goalLearnPeople: action goal containing a name and an img
        :type goalLearnPeople: ros_people_mng_actions.msg/LearnPeopleFromImgGoal 
        :param timeout: maximum time to execute the face detection and learning procedure
        :type timeout: float
        """
        try:
            rospy.loginfo("{class_name}: ### LEARN PEOPLE ATTRIBUTES ACTION PENDING".format(class_name=self.__class__.__name__))
            # send the current goal to the action server
            self._actionLearnPeopleMeta_server.send_goal(goalLearnPeople)
            # wait action server result
            finished_before_timeout = self._actionLearnPeopleMeta_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionLearnPeopleMeta_server.get_state()
            result = self._actionLearnPeopleMeta_server.get_result()
            rospy.loginfo("{class_name}: ###### LEARN PEOPLE ATTRIBUTES ACTION END , State: %s".format(class_name=self.__class__.__name__), str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionLearnPeopleMeta_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("{class_name}: ###### LEARN PEOPLE ATTRIBUTES FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED, None

    def __detect_meta_people_from_img_topic(self, timeout):
        """
        Will call the action client to detect people in the specified image on image topic.
        Returns a GoalStatus and an action result

        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        goalMetaPeople = ProcessPeopleFromImgGoal()
        state, result = self.__detect_meta_people(goalMetaPeople, timeout)
        return state, result

    def __detect_meta_people_from_img_path(self, img_path, timeout):
        """
        Will call the action client to detect people in the specified image.
        Returns a GoalStatus and an action result

        :param img_path: image path where the system will find the image to analyze.
        :type img_path: string
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        img_loaded1 = cv2.imread(img_path)
        msg_im1 = self._bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")
        goalMetaPeople = ProcessPeopleFromImgGoal(img=msg_im1)
        state, result = self.__detect_meta_people(goalMetaPeople, timeout)
        return state, result

    def __detect_meta_people(self, goalMetaPeople, timeout):
        """
        Action Client which will send a perception order to start a people detection on the image in the specified goal.
        Returns a GoalStatus and an action result

        :param goalMetaPeople: Action goal which contains the image to analyze
        :type goalMetaPeople: ros_people_mng_actions.msg/ProcessPeopleFromImgGoal
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        try:
            rospy.loginfo("{class_name}: ### DETECT META PEOPLE ACTION PENDING".format(class_name=self.__class__.__name__))
            # send the current goal to the action server
            self._actionMultiplePeopleDetection_server.send_goal(goalMetaPeople)
            # wait action server result
            finished_before_timeout = self._actionMultiplePeopleDetection_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionMultiplePeopleDetection_server.get_state()
            result = self._actionMultiplePeopleDetection_server.get_result()
            rospy.loginfo("{class_name}: ###### DETECT META PEOPLE ACTION END , State: %s".format(class_name=self.__class__.__name__), str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionMultiplePeopleDetection_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("{class_name}: ###### DETECT META PEOPLE FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        return GoalStatus.ABORTED, None

    def __get_people_name_from_img_topic(self, timeout):
        """
        Will call the action client to get people names in the specified image on image topic.
        Returns a GoalStatus and an action result

        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        goalPeopleName = GetPeopleNameFromImgGoal()
        state, result = self.__get_people_name(goalPeopleName, timeout)
        return state, result

    def __get_people_name_from_img_path(self, img_path, timeout):
        """
        Will call the action client to get people names in the specified image with the image path.
        Returns a GoalStatus and an action result
        :param img_path: path of the image to analyze
        :type img_path: string
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        img_loaded = cv2.imread(img_path)
        img_msg = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
        goalPeopleName = ProcessPeopleFromImgGoal(img=img_msg)
        state, result = self.__get_people_name(goalPeopleName, timeout)
        return state, result

    def __get_people_name(self, goalPeopleName, timeout):
        """
        Action client which will send a perception order to start a people detection on the image specified in the goal and get the people names.
        Returns a GoalStatus and an action result

        :param goalPeopleName: ActionGoal containing the image to analyze
        :type goalPeopleName: ros_people_mng_actions.msg/GetPeopleNameFromImgGoal
        :param timeout: maximum time to execute the detection procedure
        :type timeout: float
        """
        try:
            rospy.loginfo("{class_name}: ### GET PEOPLE NAME ACTION PENDING".format(class_name=self.__class__.__name__))
            # send the current goal to the action server
            self._actionGetPeopleName_server.send_goal(goalPeopleName)
            # wait action server result
            finished_before_timeout = self._actionGetPeopleName_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionGetPeopleName_server.get_state()
            result = self._actionGetPeopleName_server.get_result()
            rospy.loginfo("{class_name}: ###### GET PEOPLE NAME ACTION END , State: %s".format(class_name=self.__class__.__name__), str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionGetPeopleName_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("{class_name}: ###### GET PEOPLE NAME FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
        except Exception as e:
            rospy.logerr("{class_name}: Service min_front_value_srv could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

        return GoalStatus.ABORTED, None

    def __reset_people_meta_info_map(self):
        """
        Service client which will send a perception order to reset the people infos the system had learnt so far.
        Returns a GoalStatus and a service result
        """
        try:
            result = self._resetPeopleMetaInfoMapSP()
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service reset_people_meta_info_map_srv could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service min_front_value_srv could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __wait_for_door_to_open(self, check_freq, min_dist):
        """
        Check the min front value at a given frequency.

        :param check_freq: Frequency at which to check door opening
        :type check_freq: float
        :return: True if detected door opening without error, False otherwise
        """
        ##
        try:
            rate = rospy.Rate(check_freq)
            current_distance = 0.0
            while not rospy.is_shutdown() and current_distance < min_dist:
                current_distance = self._minFrontValueSP().value
                rate.sleep()
            rospy.loginfo("{class_name}: ************ DOOR IS OPENED ! ****************".format(class_name=self.__class__.__name__))
            return GoalStatus.SUCCEEDED, True
        except rospy.ServiceException as e:
            rospy.logerr("{class_name}: Service min_front_value_srv could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("{class_name}: Service min_front_value_srv could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
