cmake_minimum_required(VERSION 2.8.3)
project(general_mng)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robocup_msgs
  navigation_manager
  tts_hri
  dialogue_hri_actions
  object_management
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES general_mng
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


