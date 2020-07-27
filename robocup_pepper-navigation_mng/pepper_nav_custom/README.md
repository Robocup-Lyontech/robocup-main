# 1. pepper_nav_custom package

## 2. Description
This package provides tools and configuration to adapt ros navigation stack to the pepper robot


## 3. Authors
* Jacques Saraydaryan


## 4.  Context and Strategy 

## 5.  Configuration 
### costmap_common_params.yaml
below only main differences added to the costmap_common_params

```python
...
footprint: [[0.175, -0.175], [0.0897,-0.23 ], [0.009,-0.225], [-0.122,-0.15], [-0.24,0.0], [-0.122,0.15], [0.009,0.225], [0.0897,0.23], [0.175,0.175]]
...

obstacle_layer:
  enabled:              true
  max_obstacle_height:  2.0
  unknown_threshold:    15
  mark_threshold:       0
  combination_method:   1
  track_unknown_space:  true    
  obstacle_range: 2.5           
  raytrace_range: 3.0
  origin_z: 0.0
  z_resolution: 0.1
  z_voxels: 10
  publish_voxel_map: false
  observation_sources:  depth_pcl scan2 scan3 #use different scan to prevent collision and ensure obstacle cleaning
  
  # source coming from the RGBD camera of the robot, due to the poor data quality obstacle are adding only on range of 0.75m
  depth_pcl:
    data_type: PointCloud2
    topic:  /points
    marking: true
    clearing: true
    min_obstacle_height: 0.2 
    max_obstacle_height: 2.0
    observation_persistence: 0.5
    obstacle_range: 0.75 
    raytrace_range: 2.0
    inf_is_valid: false
    
# source coming from the robot laser, only laser below 2.0m are taken into account (above lead to lots of false positive)
  scan2:
    data_type: LaserScan
    topic: /pepper_robot/naoqi_driver/laser 
    marking: true
    clearing: true
    obstacle_range: 2.0  #to remove if needed or set to 1.5 to be safe, greater distance lead to faster move but with more risk of collision
    inf_is_valid: true
    min_obstacle_height: -0.15
    max_obstacle_height: 2.0

# source coming from the robot laser, only use to clean obstacle on 3.0 range
scan3:
    data_type: LaserScan
    topic: /pepper_robot/naoqi_driver/laser 
    marking: false
    clearing: true
    inf_is_valid: true
    min_obstacle_height: -0.15 
    max_obstacle_height: 2.0
    
...
```
