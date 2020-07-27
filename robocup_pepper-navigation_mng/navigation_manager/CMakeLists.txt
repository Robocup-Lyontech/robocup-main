cmake_minimum_required(VERSION 2.8.3)
project(navigation_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  robocup_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  actionlib_msgs
  message_generation
  actionlib_msgs 
  actionlib
)



## Generate actions in the 'action' folder
 add_action_files(
   FILES
   NavMng.action
 )
 generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_manager
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)
