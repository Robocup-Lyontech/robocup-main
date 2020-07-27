
# navigation_mng


## 1. Description

This package holds a set of features for navigation. It includes :
  * (deprecated) Convert PCL to laser: depthimage_to_laserscan-kinetic-devel 
  * Detect the in distance in front the robot (sonar and laser): pepper_door_open,_detector
  * Set pepper parameter for navigation, apply a pose, and move head according move commands: pepper_pose_for_nav
  * Apply a custom navigation using the ros naivgation stack (including multi-sensors for costmap updating): pepper_nav_custom
  * Manage navigation strategy, define cost map clear strategy, new goal point if needed and redo cmd list is robot freeze into obstacles: [navigation_management](https://github.com/jacques-saraydaryan/robocup_pepper-navigation_mng/tree/master/navigation_manager)


## 2. Authors
* Jacques Saraydaryan

