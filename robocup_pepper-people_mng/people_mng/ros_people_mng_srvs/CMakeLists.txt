cmake_minimum_required(VERSION 2.8.3)
project(ros_people_mng_srvs)


find_package(catkin REQUIRED COMPONENTS
    rospy
    cv_bridge
    std_msgs
    sensor_msgs
    ros_people_mng_msgs
    message_generation
    ros_color_detection_msgs
)


 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   ProcessPeopleFromImg.srv
   TakePictureService.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   ros_people_mng_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_face_identification
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
