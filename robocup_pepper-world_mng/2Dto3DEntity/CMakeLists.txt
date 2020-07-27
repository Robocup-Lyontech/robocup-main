cmake_minimum_required(VERSION 2.8.3)
project(convert_2d_to_3d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  robocup_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  visualization_msgs
  tf
  darknet_ros_msgs
)

 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   get3Dfrom2D.srv
 )

  generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
 )


# Declare a catkin package                                                                                                                                
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf

)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build

# ROS nodes
add_executable(convert_2d_to_3d src/convert_2d_to_3d.cpp) 
target_link_libraries(convert_2d_to_3d ${catkin_LIBRARIES})
