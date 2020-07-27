
# navigation_manager


## 1. Description

This package is in charge of applying different navigation strategies including functionality such as :
* Retry on failure
* Costmap clear strategy
* Goal redefinition
* Redo last command if robot is into lethal cost map

This framwork allows to defined custom navigation strategies that could be call using an associated key. 


## 2. Authors
* Jacques Saraydaryan


## 3. How to quote
F. Jumel, J. Saraydaryan, R. Leber, L. Matignon, E. Lombardi, C. Wolf and O. Simonin,”Context Aware Robot Architecture, Application to the Robocup@Home Challenge”, RoboCup Symposium 2018

## 4. Navigation creation procedure
### 4.1 Create a navigation strategy file
Go to the scripts/common/navstrategy folder. Create a new file MyStrategy.py


```python
__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from move_base_msgs.msg import MoveBaseGoal
import time

class MyStrategy(AbstractNavStrategy):

    def __init__(self,actMove_base):
        AbstractNavStrategy.__init__(self,actMove_base)
        #register clear costmap services



    def goto(self, sourcePose, targetPose):
        #Create Goal action Message
        current_goal = MoveBaseGoal()
        current_goal.target_pose.pose=targetPose       
        current_goal.target_pose.header.frame_id = 'map'
        current_goal.target_pose.header.stamp = rospy.Time.now()

        #Start global Timer
        self.startTimeWatch()
	    
        # check if global retry and global timer are not trigged
        while (self._retry_nb < self._retry_max_nb) and (not self._timeout_checker):

            # Start the robot toward the next location
            self._actMove_base.send_goal(current_goal)

            #launch navigation
            isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            current_action_state=self._actMove_base.get_state()


            if isActionResultSuccess and current_action_state==3 :
                rospy.loginfo('Navigation_Management: action state:'+ str(self._actMove_base.get_state()))
                rospy.loginfo('Navigation_Management :Goal Successfully achieved: ' + str(current_goal).replace("\n",""))

                #Reset current strategy parameters
                self.reset()
                return True
            else:
                rospy.logwarn('Goal FAILURE (waiting '+str(self._maxWaitTimePerGoal)+'): ' + str(current_goal).replace("\n",""))
                rospy.logwarn('Retrying (current retryNb:'+str(self._retry_nb)+', max retry'+str(self._retry_max_nb)+')')
                self._retry_nb=self._retry_nb+1
        rospy.logwarn('Goal FAILURE until retry and clearing, returning : [' + str(current_goal).replace("\n","")+']')
        self.reset()
        return False

        
```

the goto method is called when a naivation is asked, in this example we only retry in case of failure or timeout.

### 4.2 Update the NavigationManager.py file

```python
...
from common.navstrategy.MyScenario import MyScenario
...

    def configure(self):
       ...
        self._navigationStrategyMaps={
                    "Simple":SimplyGoNavStrategy(self._actMove_base),
                    "Retry":GoAndRetryNavStrategy(self._actMove_base),
                    "CleanAndRetry":GoCleanRetryNavStrategy(self._actMove_base),
                    "CleanRetryReplay":GoCleanRetryReplayLastNavStrategy(self._actMove_base),
                    "CRRCloseToGoal":GoCRRCloseToGoal(self._actMove_base)
                    
                    "MyScenario":MyScenario(self._actMove_base)
                    
                    }
       ...
``` 

## 5. Node
### 5.1 Subscribed Topics
* gm_bus_command ([robocup_msgs/gm_bus_msg](https://github.com/jacques-saraydaryan/robocup_pepper-robocup_msgs/blob/master/msg/gm_bus_msg.msg)):
Use to get order from [general manager](https://github.com/jacques-saraydaryan/robocup_pepper-general_mng) (if topic used)


### 5.2 Publish Topics
* gm_bus_command ([robocup_msgs/gm_bus_msg](https://github.com/jacques-saraydaryan/robocup_pepper-robocup_msgs/blob/master/msg/gm_bus_msg.msg)):
Use to give feedback to [general manager](https://github.com/jacques-saraydaryan/robocup_pepper-general_mng) (if topic used)


### 5.3 Subscribed service
* get_InterestPoint ([map_manager/getitP_service](https://github.com/jacques-saraydaryan/robocup_pepper-world_mng/blob/master/map_manager/srv/getitP_service.srv)):
Use to translate given interest point key into map coodinates

### 5.4 Action

* NavMng.action:
Use to get order from [general manager](https://github.com/jacques-saraydaryan/robocup_pepper-general_mng) (if action used)
```
#goal definition
string action   # type of asked action NT:  180° turn, NP: natigation to point according to the given strategy
string itP      # Interest point key, use the map_manager service get_InterestPoint to get coordinates
string navstrategy # Navigation strategy key
geometry_msgs/Point itP_point # target coordinates if itP is not denied

---
#result definition
int32 result
---
#feedback
int32 feedback

```



