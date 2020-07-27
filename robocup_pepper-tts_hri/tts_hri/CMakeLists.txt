cmake_minimum_required(VERSION 2.8.3)
project(tts_hri)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    rospy 
    sensor_msgs 
    std_msgs 
    tf 
    dynamic_reconfigure 
    robocup_msgs
    actionlib_msgs
    message_generation
    actionlib_msgs 
    actionlib
    )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   TtsHri.action
 )
 generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)