
# map_manager


## 1. Description

Store interest points coordinates (location and orientation) and associate robot information (hand, head position)  into the map reference


## 2. Authors
* Jacques Saraydaryan


## 3. How to quote
F. Jumel, J. Saraydaryan, R. Leber, L. Matignon, E. Lombardi, C. Wolf and O. Simonin,”Context Aware Robot Architecture, Application to the Robocup@Home Challenge”, RoboCup Symposium 2018


## 4. Node


### 4.2  Services
#### 4.2.1 load_InterstPoint (std_msgs/Empty)
load the registred interest points sav into the configuration folder

#### 4.2.2 save_InterestPoint ([map_manager/saveitP_service](./srv/saveitP_service.srv))
Save a given coordinates as an interest point into the config folder

#### 4.2.3 get_InterestPoint ([map_manager/getitP_service](./srv/getitP_service.srv))
Return a [robocup_msgs/InterestPoint](https://github.com/jacques-saraydaryan/robocup_pepper-robocup_msgs/blob/master/msg/InterestPoint.msg) for a given key

#### 4.2.4 save_BaseLinkInterestPoint ([map_manager/saveitP_base_link_service](./srv/saveitP_base_link_service.srv))
Save the current robot base_link TF as an interest point into the config folder


#### 4.2.5 activate_InterestPointTF ([map_manager/activateTF_service](./srv/activateTF_service.srv))

Publish TF of loaded Interest point


### 4.3 Params

#### 4.3.1 confPath
Location of the config folder where interest points are loaded and saved


## 5. MapTools.py
### 5.1 Description
MapTools.py is tools allowing to save interest point from rviz using the "2D Nav Goal" rviz tool.

### 5.2 How to use
Start the MapTools.py node

```
rosrun map_manager MapTools.py _confPath:="/tmp/ITs"

```

Publish an already created map


```
rosrun map_server map_server myMap.yaml

```

Start Rviz

```
rviz

```

Add the map topic and
use the "2D Nav Goal" rviz tool to defined Interest points


