cmake_minimum_required(VERSION 2.8.3)
project(dialogue_hri_srvs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs tf dynamic_reconfigure robocup_msgs)


 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   ReleaseArms.srv
   GoToCarryPose.srv
   PointAt.srv
   MoveArmHand.srv
   MoveSound.srv
   MoveTurn.srv
   TakePicture.srv
   TurnToInterestPoint.srv

 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   robocup_msgs
 )

catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
