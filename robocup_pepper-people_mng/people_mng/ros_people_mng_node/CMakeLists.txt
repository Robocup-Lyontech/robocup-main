cmake_minimum_required(VERSION 2.8.3)
project(ros_people_mng_node)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    actionlib_msgs 
    actionlib
    ros_people_mng_msgs
    ros_people_mng_srvs
    ros_people_mng_actions
)



catkin_package(
   # INCLUDE_DIRS include
#  LIBRARIES people_face_identification
 # CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)