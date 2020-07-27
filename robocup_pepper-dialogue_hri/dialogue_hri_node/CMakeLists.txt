cmake_minimum_required(VERSION 2.8.3)
project(dialogue_hri_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy 
  sensor_msgs 
  std_msgs
  actionlib
  actionlib_msgs 
  dialogue_hri_srvs
  
  )


 #Generate services in the 'srv' folder
 #add_service_files(
 #  FILES
 #  TODO.srv
 #)

## Generate added messages and services with any dependencies listed here
 # generate_messages(
  #  DEPENDENCIES
   # std_msgs
 # )

catkin_package(
  #  INCLUDE_DIRS
#  LIBRARIES laser_merger
 # CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
