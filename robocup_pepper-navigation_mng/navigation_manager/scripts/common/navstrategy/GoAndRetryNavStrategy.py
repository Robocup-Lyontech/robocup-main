__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




class GoAndRetryNavStrategy(AbstractNavStrategy):

    def goto(self, sourcePose, targetPose):
        #Create Goal action Message
        current_goal = MoveBaseGoal()
        current_goal.target_pose.pose=targetPose       
        current_goal.target_pose.header.frame_id = 'map'
        current_goal.target_pose.header.stamp = rospy.Time.now()
        # Start the robot toward the next location
        self._actMove_base.send_goal(current_goal)

        #Start global Timer
        self.startTimeWatch()
	    
        # check if global retry and global timer are not trigged
        while (self._retry_nb <= self._retry_max_nb) and ( not self._timeout_checker):
            isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            current_action_state=self._actMove_base.get_state()
                #if isActionResultSuccess and self._actMove_base.get_state()!= self.MOVE_BASE_ACTION_STATE_FAILURE:
            if isActionResultSuccess and current_action_state ==3 :
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
                self._retry_nb=self._retry_nb+1
           
        self.reset()
        return False
        