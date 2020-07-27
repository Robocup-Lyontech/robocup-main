__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




class SimplyGoNavStrategy(AbstractNavStrategy):

    def goto(self, sourcePose, targetPose):
        #Create Goal action Message
        current_goal = MoveBaseGoal()
        current_goal.target_pose.pose=targetPose       
        current_goal.target_pose.header.frame_id = 'map'
        current_goal.target_pose.header.stamp = rospy.Time(0)
        # Start the robot toward the next location
        self._actMove_base.send_goal(current_goal)

        isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))

        current_action_state=self._actMove_base.get_state()
                
        if isActionResultSuccess and current_action_state == 3 :
            rospy.loginfo('Navigation_Management: action state:'+ str(self._actMove_base.get_state()))
            rospy.loginfo('Navigation_Management :Goal Successfully achieved: ' + str(current_goal).replace("\n",""))
            return True
        else:
            rospy.logwarn('Goal FAILURE (waiting '+str(self._maxWaitTimePerGoal)+'): ' + str(current_goal).replace("\n",""))
            return False
        