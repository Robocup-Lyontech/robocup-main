cmake_minimum_required(VERSION 2.8.3)
project(robocup_msgs)


find_package(catkin REQUIRED COMPONENTS
rospy
cv_bridge
std_msgs
sensor_msgs
geometry_msgs
message_generation
)


add_message_files(FILES
  Box.msg
  Entity.msg
  EntityList.msg
  Entity2D.msg
  Entity2DList.msg
  InterestPoint.msg
  Action.msg
  NavigationAction.msg
  gm_bus_msg.msg
  OAction.msg
  OrganizedActionList.msg
)

 #Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
   # Or other packages containing msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_face_identification
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

