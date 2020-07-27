__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from actionlib_msgs.msg import GoalID
import time
from tf import TransformListener
import actionlib

from common.tools.Lifo import Lifo
from CmdTwist import CmdTwist



class GoCleanRetryReplayLastNavStrategy(AbstractNavStrategy):
    _twistBufferSize=100
    _twistLifo=[]
    _isReversePathActivated=False
    _globalCostMap=''
    _localCostMap=''
    _tflistener=''
    _maxCostMapTolerance=70
    _stopCurrentNav=False
    _current_goalhandle=False
    _isReplyLastCmdActivated=True

    def __init__(self,actMove_base):
        AbstractNavStrategy.__init__(self,actMove_base)
        
        
        self._tflistener = TransformListener()

        self._twistLifo=Lifo(self._twistBufferSize)
        
        
        #register clear costmap services

        try:
            rospy.wait_for_service('/static_map',5)
            rospy.loginfo("end service static_map wait time")
            self._getMap = rospy.ServiceProxy('static_map', GetMap)
            # reset_costmap()
        except Exception as e:
            rospy.loginfo("Service static_map call failed: %s" % e)

        try:
            rospy.wait_for_service('/move_base/clear_costmaps',5)
            rospy.loginfo("end service all wait time")
            self._reset_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            # reset_costmap()
        except Exception as e:
            rospy.loginfo("Service clear cost maps call failed: %s" % e)

        
        self._map_pub=rospy.Publisher('map',OccupancyGrid,queue_size=1)
 
        #FIME TO BE UPDATED WITH REAL TOPIC NAME OF PEPPER
        #CAUTION naoqi could block cmd if collision risk...

        self._twist_sub=rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)
        #self._twist_sub=rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.twist_callback)
        #self._twist_teleop_sub=rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, self.twist_callback)


        self._twist_pub=rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)

        #FIME TO BE UPDATED WITH REAL TOPIC NAME OF PEPPER
        self._globalCostMap_sub=rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.globalCostMap_callback)
        self._localCostMap_sub=rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.localCostMap_callback)




    def goto(self, sourcePose, targetPose):
        ##NEED TO Make stuff into another thread


        #Start global Timer
        self.startTimeWatch()

        self.stopAll()
        self._stopCurrentNav=False

        #Create Goal action Message
        current_goal = MoveBaseGoal()
        current_goal.target_pose.pose=targetPose       
        current_goal.target_pose.header.frame_id = 'map'
        # current_goal.target_pose.header.stamp = rospy.Time(0)
        current_goal.target_pose.header.stamp = rospy.Time.now()

        rospy.logwarn("{class_name} : goal sent to move base : %s".format(class_name=self.__class__.__name__),str(current_goal))
	    
        # check if global retry and global timer are not trigged
        while (self._retry_nb < self._retry_max_nb) and (not self._timeout_checker) and (not self._stopCurrentNav) and (not rospy.is_shutdown()):

            # Start the robot toward the next location
            rospy.loginfo("{class_name} : SENDING GOAL NOW : %s".format(class_name=self.__class__.__name__),str(rospy.Time.now()))
            self._current_goalhandle = self._actMove_base.send_goal(current_goal)

            #launch navigation
            rospy.logwarn("{class_name} : TIMEOUT JACQUES : %s".format(class_name=self.__class__.__name__),str(self._maxWaitTimePerGoal))
            self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            rospy.logwarn("{class_name} : AFTER WAIT FOR RESULT".format(class_name=self.__class__.__name__))

            current_action_state=self._actMove_base.get_state()
            rospy.loginfo("{class_name} : DEBUG CURRENT ACTION STATE : %s".format(class_name=self.__class__.__name__),str(current_action_state))

                #if isActionResultSuccess and self._actMove_base.get_state()!= self.MOVE_BASE_ACTION_STATE_FAILURE:
            if  current_action_state==3 :
                rospy.loginfo('Navigation_Management: action state:'+ str(self._actMove_base.get_state()))
                rospy.loginfo('Navigation_Management :Goal Successfully achieved: ' + str(current_goal).replace("\n",""))
                #rospy.loginfo('Wait now extected duration: ' + str(data.action.waitTime))
                #Sleep the expected time
                #time.sleep(data.action.waitTime)
                #rospy.loginfo('Sleep end')
                
                #Reset current strategy parameters
                self.reset()
                return True

            # elif current_action_state == 8 : 
            #     rospy.loginfo("{class_name} : action state : %s".format(class_name=self.__class__.__name__),str(current_action_state))
            #     return False


            else:
                rospy.logwarn('Navigation_Management: action state:'+ str(self._actMove_base.get_state()))
                rospy.logwarn('Goal FAILURE (waiting '+str(self._maxWaitTimePerGoal)+'): ' + str(current_goal).replace("\n",""))

                rospy.loginfo('Clear all costmaps')
                self.resetCostMaps()

                rospy.loginfo('Check if current baselink is into critical cost map')
                if self.isBaseLinkIntoGlobalCostMap():
                    rospy.logwarn('BASE LINK into critical cost map')
                    rospy.loginfo('ASK for reverse last Twist Commands')
                    self.reverseLastTwist()


                rospy.logwarn('{class_name} : Retrying (current retryNb:'.format(class_name=self.__class__.__name__)+str(self._retry_nb)+', max retry'+str(self._retry_max_nb)+')')
                self._retry_nb=self._retry_nb+1
        rospy.logwarn('{class_name} : Goal FAILURE until retry and clearing, returning : ['.format(class_name=self.__class__.__name__) + str(current_goal).replace("\n","")+']')
        
        self.stopAll()
        self.reset()
        return False

    def stopAll(self):
        self._stopCurrentNav=True
        #Create Goal action Message
        #self._actMove_base.cancel_all_goals()
        #self._actMove_base.cancel_goal()
        #rospy.Time(0)
        #rospy.get_rostime()
        rospy.logwarn("{class_name} : CANCELLING GOAL AT AND BEFORE TIME".format(class_name=self.__class__.__name__))
        self._actMove_base.cancel_goals_at_and_before_time(rospy.Time(0))
        # self._actMove_base.cancel_goals_at_and_before_time(rospy.Time.now())

        #CAUTION update the global_cost_map publish_frequency parameter to work with this wait time (e.g 2.0hz) +update frequency (e.g 2.5hz) 
        rospy.sleep(0.5)
       
        
    def resetCostMaps(self):
        try:
            # call clear all costmap before perfoming navigation
            self._reset_costmap()
        except Exception as e:
            rospy.loginfo("{class_name} : Service clear costmap call failed: %s".format(class_name=self.__class__.__name__), str(e))
        #CAUTION Resending a map could cause robot localisation failure
        #Get map and republish to be sure that costmap are uptades again
        #try:
        #    current_map=self._getMap()
        #except Exception as e:
        #    rospy.loginfo("Service static map call failed: %s" % e)
        # 
        #self._map_pub.publish(current_map.map)
        #rospy.sleep(5)

    def twist_callback(self,msg):
        if not self._isReversePathActivated:
            cmd=CmdTwist(msg)
            if self._twistLifo.size() >0 :
                #do not register 2 times stop message
                if self.isStopCmdTwist(msg) and self.isStopCmdTwist(self._twistLifo.get(self._twistLifo.size()-1).getTwistCmd()):
                    return
                #set stop time of last cmd
                self._twistLifo.get(self._twistLifo.size()-1).setStopTime(time.time())
            self._twistLifo.put(cmd)

    def reverseLastTwist(self):
        try:
            if not self._isReplyLastCmdActivated:
                return
            self._isReversePathActivated=True
            #set last twist cmd stop time with current date
            try:
                self._twistLifo.get(self._twistLifo.size()-1).setStopTime(time.time())
            except Exception as e:
                rospy.loginfo("Unable to reverse twist cmd: %s" % e)

            for i in range(self._twistLifo.size()):
                
                twist_cmd=self._twistLifo.pop()
                if self.isStopCmdTwist(twist_cmd.getTwistCmd()):
                    duration=0.1
                else:
                    duration=twist_cmd.duration()
                #revert the twist cmd linear and angular 
                reversed_twist_cmd=twist_cmd.reverse()
                self._twist_pub.publish(reversed_twist_cmd)
                #CAUTION need to register also the time elapsed betweed twist cmd...
                time.sleep(duration)
                
        finally:
            self._isReversePathActivated=False


    def isStopCmdTwist(self,msg):
        if abs(msg.linear.x) == 0 and abs(msg.linear.y) == 0 and abs(msg.linear.z) == 0 and abs(msg.angular.x) == 0 and abs(msg.angular.y) == 0 and abs(msg.angular.z) == 0 :
            return True
        else:
            return False 


    def isBaseLinkIntoGlobalCostMap(self):
        now = rospy.Time(0)
        self._tflistener.waitForTransform("/map", "/base_link", now, rospy.Duration(2.0))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", now)
        rospy.loginfo("{class_name} : GLOBAL COSTMAP DEBUG".format(class_name=self.__class__.__name__))
        global_cost_value=self.getCostMapValue(trans[0],trans[1],self._globalCostMap)
        rospy.loginfo("{class_name} : global cost value isBaseLinkIntoCostMap : %s".format(class_name=self.__class__.__name__),str(global_cost_value))

        rospy.loginfo("{class_name} : LOCAL COSTMAP DEBUG".format(class_name=self.__class__.__name__))
        local_cost_value=self.getCostMapValue(trans[0],trans[1],self._localCostMap)
        rospy.loginfo("{class_name} : local cost value isBaseLinkIntoCostMap : %s".format(class_name=self.__class__.__name__),str(local_cost_value))


        if global_cost_value >= self._maxCostMapTolerance or local_cost_value >= self._maxCostMapTolerance:
            return True
        else:
            return False
            
    def isPtIntoCostMap(self,x,y):
        rospy.loginfo("{class_name} : Is point into cost map ?".format(class_name=self.__class__.__name__))
        global_cost_value=self.getCostMapValue(x,y,self._globalCostMap)
        rospy.loginfo("{class_name} : global cost value isPtIntoCostMap : %s".format(class_name=self.__class__.__name__),str(global_cost_value))

        #FIME need to adjust coord for local costmap
        #local_cost_value=self.getCostMapValue(x,y,self._localCostMap)
        local_cost_value=0
        if global_cost_value >= self._maxCostMapTolerance or local_cost_value >= self._maxCostMapTolerance:
            return True
        else:
            return False




    def getCostMapValue(self,x,y,map):

        rospy.loginfo("{class_name} : X : %s".format(class_name=self.__class__.__name__),str(x))
        rospy.loginfo("{class_name} : Y : %s".format(class_name=self.__class__.__name__),str(y))

        rospy.loginfo("{class_name} : MAP ORIGIN X : %s".format(class_name=self.__class__.__name__),str(map.info.origin.position.x))
        rospy.loginfo("{class_name} : MAP ORIGIN Y : %s".format(class_name=self.__class__.__name__),str(map.info.origin.position.y))

        rospy.loginfo("{class_name} : MAP RESOLUTION %s".format(class_name=self.__class__.__name__),str(map.info.resolution))
        rospy.loginfo("{class_name} : MAP WIDTH %s".format(class_name=self.__class__.__name__),str(map.info.width))
        rospy.loginfo("{class_name} : MAP HEIGHT %s".format(class_name=self.__class__.__name__),str(map.info.height))

        try:

            grid_x = int(round((x - map.info.origin.position.x) / map.info.resolution))
            grid_y = int(round((y - map.info.origin.position.y) / map.info.resolution))

            rospy.logwarn("{class_name} : GRID_X %s".format(class_name=self.__class__.__name__),str(grid_x))
            rospy.logwarn("{class_name} : GRID_Y %s".format(class_name=self.__class__.__name__),str(grid_y))

            #CAUTION not sure of behavior if map is not square e.g width != height...
            index_y = int(round(grid_y * map.info.width))
            #FIXME TO BE CHECKED!!!
            rospy.logwarn("{class_name} : MAP LEN : %s".format(class_name=self.__class__.__name__),str(len(map.data)))
            rospy.logwarn("{class_name} : INDEX_Y %s".format(class_name=self.__class__.__name__),str(index_y))


            return map.data[grid_x+index_y]
        except Exception as e:
            rospy.logerr("{class_name} : ERROR TO GET COST MAP VALUE : %s . COST MAP VALUE SET TO 0".format(class_name=self.__class__.__name__),str(e))
            return 0


    def globalCostMap_callback(self,msg):
        self._globalCostMap=msg

    def localCostMap_callback(self,msg):
         self._localCostMap=msg




    
            