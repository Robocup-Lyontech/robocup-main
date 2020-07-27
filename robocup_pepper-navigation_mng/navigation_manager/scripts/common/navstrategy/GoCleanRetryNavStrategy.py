__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import time





class GoCleanRetryNavStrategy(AbstractNavStrategy):

    def __init__(self,actMove_base):
        AbstractNavStrategy.__init__(self,actMove_base)
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


    def goto(self, sourcePose, targetPose):
        #Create Goal action Message
        current_goal = MoveBaseGoal()
        current_goal.target_pose.pose=targetPose       
        current_goal.target_pose.header.frame_id = 'map'
        current_goal.target_pose.header.stamp = rospy.Time(0)

        #Start global Timer
        self.startTimeWatch()
	    
        # check if global retry and global timer are not trigged
        while (self._retry_nb < self._retry_max_nb) and (not self._timeout_checker):

            # Start the robot toward the next location
            self._actMove_base.send_goal(current_goal)

            #launch navigation
            isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            current_action_state=self._actMove_base.get_state()


                #if isActionResultSuccess and self._actMove_base.get_state()!= self.MOVE_BASE_ACTION_STATE_FAILURE:
            if isActionResultSuccess and current_action_state==3 :
                rospy.loginfo('Navigation_Management: action state:'+ str(self._actMove_base.get_state()))
                rospy.loginfo('Navigation_Management :Goal Successfully achieved: ' + str(current_goal).replace("\n",""))
                #rospy.loginfo('Wait now extected duration: ' + str(data.action.waitTime))
                #Sleep the expected time
                #time.sleep(data.action.waitTime)
                #rospy.loginfo('Sleep end')
                
                #Reset current strategy parameters
                self.reset()
                return True
            else:
                rospy.logwarn('Goal FAILURE (waiting '+str(self._maxWaitTimePerGoal)+'): ' + str(current_goal).replace("\n",""))
                rospy.logwarn('Retrying (current retryNb:'+str(self._retry_nb)+', max retry'+str(self._retry_max_nb)+')')
                rospy.loginfo('Clear all costmaps')
                self.resetCostMaps()
                self._retry_nb=self._retry_nb+1
        rospy.logwarn('Goal FAILURE until retry and clearing, returning : [' + str(current_goal).replace("\n","")+']')
        self.reset()
        return False
        
    def resetCostMaps(self):
        try:
            # call clear all costmap before perfoming navigation
            self._reset_costmap()
        except Exception as e:
            rospy.loginfo("Service clear costmap call failed: %s" % e)
        #CAUTION Resending a map could cause robot localisation failure
        #Get map and republish to be sure that costmap are uptades again
        #try:
        #    current_map=self._getMap()
        #except Exception as e:
        #    rospy.loginfo("Service static map call failed: %s" % e)
        #
        #self._map_pub.publish(current_map.map)
        #rospy.sleep(5)


            