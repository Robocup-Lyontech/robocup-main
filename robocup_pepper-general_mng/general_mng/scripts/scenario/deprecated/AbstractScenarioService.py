import rospy
from rospy.exceptions import ROSException, ROSInterruptException

from std_srvs.srv import Trigger
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from dialogue_hri_srvs.srv import MoveTurn
from dialogue_hri_srvs.srv import PointAt
from pepper_door_open_detector.srv import MinFrontValue
from dialogue_hri_srvs.srv import ReleaseArms


class AbstractScenarioService:
    HEAD_PITCH_CENTER = 0.0
    HEAD_PITCH_FOR_SPEECH_POSE = -0.30
    HEAD_PITCH_FOR_LOOK_AT_PEOPLE = -0.5
    HEAD_PITCH_FOR_LOOK_FOR_CHAIR = 0.1
    HEAD_PITCH_FOR_NAV_POSE = 0.5
    HEAD_YAW_CENTER = 0.0
    OPEN_DOOR_MIN_DISTANCE = 0.8

    _enableMoveHeadPoseService = True
    _enableMoveTurnService = True
    _enablePointAtService = False
    _enableResetPersonMetaInfoMap = False
    _enableMinFrontValueService = False
    _enableResetPersonMetaInfoMapService = False
    _enableReleaseArmsService = False

    def __init__(self):
        pass

    def configure_intern(self):
        # # Connect to move_head_service service
        # if self._enableMoveHeadPoseService:
        #     rospy.loginfo("Connecting to the move_head_pose_srv service...")
        #     self._moveHeadPoseSP = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        #     try:
        #         move_head_pose_srv_is_up = rospy.wait_for_service('move_head_pose_srv', timeout=10.0)
        #         rospy.loginfo("Connected to the move_head_pose_srv service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the move_head_pose_srv service.")
        #
        # # Connect to move_turn_service service
        # if self._enableMoveTurnService:
        #     rospy.loginfo("Connecting to the move_turn_srv service...")
        #     self._moveTurnSP = rospy.ServiceProxy('move_turn_service', MoveTurn)
        #     try:
        #         move_turn_srv_is_up = rospy.wait_for_service('move_turn_service', timeout=10.0)
        #         rospy.loginfo("Connected to the move_turn_service service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the move_turn_service service.")
        #
        # # Connect to point_at service
        # if self._enablePointAtService:
        #     rospy.loginfo("Connecting to the point_at service...")
        #     self._pointAtSP = rospy.ServiceProxy('point_at', PointAt)
        #     try:
        #         point_at_srv_is_up = rospy.wait_for_service('point_at', timeout=10.0)
        #         rospy.loginfo("Connected to the point_at service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the point_at service.")

        # Connect to person Mat info erasing service
        # if self._enableResetPersonMetaInfoMapService:
        #     rospy.loginfo("Connecting to the reset_people_meta_info_map_srv service...")
        #     self._resetPeopleMetaInfoMapSP = rospy.ServiceProxy('reset_people_meta_info_map_srv', Trigger)
        #     try:
        #         reset_people_meta_info_map_srv_is_up = rospy.wait_for_service('reset_people_meta_info_map_srv', timeout=10.0)
        #         rospy.loginfo("Connected to the reset_people_meta_info_map_srv service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the reset_people_meta_info_map_srv service.")
        # Connect to person Mat info erasing service
        # if self._enableReleaseArmsService:
        #     rospy.loginfo("Connecting to the release_arms service...")
        #     self._releaseArmsSP = rospy.ServiceProxy('release_arms', ReleaseArms)
        #     try:
        #         release_arms_srv_is_up = rospy.wait_for_service('release_arms', timeout=10.0)
        #         rospy.loginfo("Connected to the release_arms service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the release_arms service.")


        # Connect to min_front_value_srv service
        # if self._enableMinFrontValueService:
        #     rospy.loginfo("Connecting to the min_front_value_srv service...")
        #     self._minFrontValueSP = rospy.ServiceProxy('min_front_value_srv', MinFrontValue)
        #     try:
        #         min_front_value_srv_is_up = rospy.wait_for_service('min_front_value_srv', timeout=10.0)
        #         rospy.loginfo("Connected to the min_front_value_srv service.")
        #     except (ROSException, ROSInterruptException) as e:
        #         rospy.logwarn("Unable to connect to the min_front_value_srv service.")

    # def moveheadPose(self, pitch_value, yaw_value, track):
    #     try:
    #         return self._moveHeadPoseSP(pitch_value, yaw_value, track)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service move_head_pose_srv could not process request: {error}".format(error=e))
    #
    # def moveTurn(self, rad):
    #     try:
    #         return self._moveTurnSP(rad)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service move_turn_service could not process request: {error}".format(error=e))
    #
    # def pointAt(self, x, y, z, head, arm, duration):
    #     try:
    #         return self._pointAtSP(x, y, z, head, arm, duration)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service point_at could not process request: {error}".format(error=e))

    # def resetPeopleMetaInfoMap(self):
    #     try:
    #         return self._resetPeopleMetaInfoMapSP()
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service reset_people_meta_info_map_srv could not process request: {error}".format(error=e))
    #
    # def waitForDoorToOpen(self, check_freq=10.0):
    #     """
    #     Check the min front value at a given frequency.
    #
    #     :param check_freq: Frequency at which to check door opening
    #     :type check_freq: float
    #     :return: True if detected door opening without error, False otherwise
    #     """
    #     ##
    #     try:
    #         rate = rospy.Rate(check_freq)
    #         current_distance = 0.0
    #         while not rospy.is_shutdown() and current_distance < self.OPEN_DOOR_MIN_DISTANCE:
    #             current_distance = self._minFrontValueSP().value
    #             rate.sleep()
    #         rospy.loginfo("************ DOOR IS OPENED ! ****************")
    #         return True
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service min_front_value_srv could not process request: {error}".format(error=e))
    #         return False

    # def releaseArms(self):
    #     try:
    #         return self._releaseArmsSP(0.6)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service release_arms could not process request: {error}".format(error=e))
