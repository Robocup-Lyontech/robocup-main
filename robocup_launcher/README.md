# Robocup launcher



## Prepare robocup settings

### Bash 1 (classic term)
```bash
DISPLAY=:0 xhost si:localuser:root
cd ~/ros/robocup_2021_ws/src/robocup-simulation-starter/lyontech_confs
docker-compose -f docker-compose.lyontech-nvidia.yml up
```

### Bash 2 (classic term)
```bash
docker start sweet_dewdney
docker start some-postgis
```

### Bash 3 (terminator)
```bash
# Alt-l --> Robocup
# Alt-a --> Broadcast
cd ~/ros/robocup_2021_ws/src/robocup-simulation-starter/lyontech_confs
source ./set-rosmaster.sh
# Alt-o --> Stop Broadcast

# Sub-terminal Darknet
roslaunch object_management merge_topics_to_one.launch

# Sub-terminal robocup-main
roslaunch robocup_launcher common_stuff.launch # NEW, replace other launches. Details in section COMMON STUFF below
roslaunch general_mng general_manager.launch
roslaunch pmb2_apps palbator_moveit_control.launch

# Sub-terminal Rviz + lancement 
rostopic pub /gm_start std_msgs/String "data: 'Robocup_simu_scenario'"

```



## Package Description

### COMMON STUFF

```bash
roslaunch robocup_launcher common_stuff.launch
```

Includes :
    - World Manager
    - Navigation Manager  


Nodes :
    - namo_scan.py
    - MessageParser.py
    - Rviz


### Message Parser

[Message Parser README](script/README.md)