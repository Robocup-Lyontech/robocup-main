__author__ = 'Jacques Saraydaryan'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetPlan
from actionlib_msgs.msg import GoalID
import time
import tf
import actionlib

from common.tools.Lifo import Lifo
from common.tools import MathToolbox
from CmdTwist import CmdTwist
from GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy



class GoCRRCloseToGoal(GoCleanRetryReplayLastNavStrategy,object):
    MAKE_PLAN_TOLERANCE=0.2 #in meter
    MAX_NB_RETRY=2
    MAX_TIME_ELAPSED_PER_GOAL=45
    MAX_NB_RETRY_FOR_VALID_MAKE_PLAN=6
    MAX_NB_RETRY_OVERALL=4
    TOLERANCE_TO_OBJECTIVE_STEP=0.8 # in meter
    SLEEP_AFTER_CLEAR_COSTMAP=1.0 # in second

    def __init__(self,actMove_base):
        GoCleanRetryReplayLastNavStrategy.__init__(self,actMove_base)
        self.configure()

        self._tflistener = tf.TransformListener()
        #/move_base/NavfnROS/make_plan
        try:
            rospy.wait_for_service('/move_base/make_plan',5)
            rospy.loginfo("end service make_plan wait time")
            self._makePlan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        except Exception as e:
            rospy.logwarn("---INIT-- Service make_plan call failed: %s" % e)

    def configure(self):
        self.setMaxNbRetry(self.MAX_NB_RETRY)
        #self.setMaxTimeElapsed(60)
        self.setMaxTimeElapsedPerGoal(self.MAX_TIME_ELAPSED_PER_GOAL)
        self._maxNbRetryForValidMakePlan=self.MAX_NB_RETRY_FOR_VALID_MAKE_PLAN
        self._nbRetryForValidMakePlan=0
        #FIXME disable replay last command
        self._isReplyLastCmdActivated=False

    def reset(self):
        super(GoCRRCloseToGoal, self).reset()
        self._nbRetryForValidMakePlan=0


    def goto(self, sourcePose, targetPose):
        rospy.loginfo("-------------- GoCRRCloseToGoal  NEW NAVIGATION ORDER [%s,%s] -----------------",str(targetPose.position.x),str(targetPose.position.y))
        # to do make plan if no plan
        # select new goal close to initial goal and loop

        #FIXME TO REMOVE ONLY FOR TESTs
        #robotPose=self.getRobotPose()
        #robotPose.pose.position.x=0
        #robotPose.pose.position.y=0
        #newGoal=self.processNewGoal(robotPose.pose,targetPose,.5)
        #isvalidPlan=self.isValidPlan(robotPose,targetPose)
        #super(GoCRRCloseToGoal, self).goto(sourcePose, newGoal)
        #targetPose.position.x=0
        #targetPose.position.y=0
        #END FIXME

        #Save original goal
        original_goal=targetPose

        #reset costmap before checking plan is valid
        self.resetCostMaps()

        #sleep to wait cost map agin (global) CAUTION global cost map frequency need to be >1hz
        rospy.sleep(self.SLEEP_AFTER_CLEAR_COSTMAP)
        
        # check first time is plan is valid
        robotPose=self.getRobotPose()
        isplanValid=self.isValidPlan(robotPose,targetPose)

        if isplanValid:
            rospy.loginfo("{class_name} : Current plan is valid".format(class_name=self.__class__.__name__))
        else:
            rospy.logwarn("Current plan is not valid...")
            newgoal=self.getValidGoal(targetPose)
            rospy.loginfo(str(newgoal))
            #if no valid plan abord
            if newgoal == None :
                rospy.logwarn("Failed to get a valid plan")
                #return False
            targetPose=newgoal
        
        #No make plan available....
        if targetPose == None :
            rospy.logwarn("Failed to get a valid plan ----------------------> TRY ORIGINAL GOAL ...........")
            targetPose=original_goal

        current_nb_newplan_recovery=0
        while current_nb_newplan_recovery < self.MAX_NB_RETRY_OVERALL and not rospy.is_shutdown():
            result= super(GoCRRCloseToGoal, self).goto(sourcePose, targetPose)
            # if navigation failed
            if not result:

                 #reset costmap before checking plan is valid
                self.resetCostMaps()

                #sleep to wait cost map agin (global) CAUTION global cost map frequency need to be >1hz
                rospy.sleep(self.SLEEP_AFTER_CLEAR_COSTMAP)

                # try to make a new plan x times
                newgoal=self.getValidGoal(targetPose)
                #if no valid plan abord
                if newgoal == None :
                    return False
                else:
                #if valid plan try again to navigate, CAUTION number of retry is limited to avoid blocking loop
                    current_nb_newplan_recovery=current_nb_newplan_recovery+1
                    targetPose=newgoal
            else:
            # if navigation success return success
                self.reset()
                return True
        # navigation failed and max retry reached
        return False
                
    def getRobotPose(self):
        self._tflistener.waitForTransform("/base_link", "/map", rospy.Time(0), rospy.Duration(5.0))
        robot_p=PoseStamped()
        robot_p.header.frame_id="/base_link"
        robot_p.pose.position.x=0
        robot_p.pose.position.y = 0
        robot_p.pose.position.z = 0
        robotPose = self._tflistener.transformPose("/map", robot_p)
        return robotPose

         #try:
         #    self._tflistener.waitForTransform("/base_link", "/map", rospy.Time.now(), rospy.Duration(2.0))
         #    robotPose = self._tflistener.transformPose("/map", "/base_link")
         #    return robotPose
         #except Exception as e:
         #    rospy.loginfo("no common frame between [/map] and [base_link] for the robot pose---------------:"+str(e))
         #    return None

   
    def processNewGoal(self,source,target,r):
        x1=source.position.x
        y1=source.position.y
        x2=target.position.x
        y2=target.position.y

        # if x1 == x2 and y1 == y2:
        #     rospy.logwarn("{class_name} : source and target have the same coordinates".format(class_name=self.__class__.__name__))
        #     return None

        a,b=MathToolbox.getLineEquation(x1,y1,x2,y2)
        goalList=MathToolbox.getLineCirclePts(a,b,x2,y2,r)
        targetedPose=Pose()

        # select the goal position
        if len(goalList)>1:
           dist1=MathToolbox.ptDistance(x1,y1,goalList[0][0],goalList[0][1])
           dist2=MathToolbox.ptDistance(x1,y1,goalList[1][0],goalList[1][1])
           if dist1>dist2:
               targetedPose.position.x=goalList[1][0]
               targetedPose.position.y=goalList[1][1]
           else:
               targetedPose.position.x=goalList[0][0]
               targetedPose.position.y=goalList[0][1]
        elif len(goalList) == 1:
            targetedPose.position.x=goalList[0][0]
            targetedPose.position.y=goalList[0][1]
        elif len(goalList)==0:
            return None

        distNewGoal=MathToolbox.ptDistance(targetedPose.position.x,targetedPose.position.y,x2,y2)
        distCurrentPose=MathToolbox.ptDistance(x1,y1,x2,y2)

        if distCurrentPose < distNewGoal:
            source.orientation=MathToolbox.computeQuaternion(x1,y1,x2,y2)
            return source
        # compute the goal quaternion
        targetedPose.orientation=MathToolbox.computeQuaternion(targetedPose.position.x,targetedPose.position.y,x2,y2)
        return targetedPose

    def getValidGoal(self,targetPose):
        _nbRetryForValidMakePlan=0
        while _nbRetryForValidMakePlan < self._maxNbRetryForValidMakePlan:
                #get current robot position
                robotPose=self.getRobotPose()

                rospy.logwarn("{class_name} : ROBOT POSE : %s".format(class_name=self.__class__.__name__),str(robotPose))
                #process new goal according tolerance
                newGoal=self.processNewGoal(robotPose.pose,targetPose,self.TOLERANCE_TO_OBJECTIVE_STEP*(_nbRetryForValidMakePlan+1))
                # if newGoal is None:
                #     return None
                isvalidPlan=self.isValidPlan(robotPose,newGoal)
                if not isvalidPlan:
                    _nbRetryForValidMakePlan=_nbRetryForValidMakePlan+1
                else:
                    return newGoal
        return None


    def isValidPlan(self,startPoseStamped,targetPose):
        # self._makePlan get the new plan and check if ok if plan ok newGoal = Goal
        try:
            start=startPoseStamped
            goal=PoseStamped()
            goal.header.frame_id = 'map'
            # goal.header.stamp = rospy.Time(0)
            goal.header.stamp = rospy.Time.now()

            goal.pose=targetPose
            tolerance=self.MAKE_PLAN_TOLERANCE
            #rospy.logwarn('before make plan')
            current_plan=self._makePlan(start,goal,tolerance)
            #rospy.loginfo(str(current_plan))
            if len(current_plan.plan.poses) == 0:
                return False
            else: 
                # if new goal into global costmap lethal zone
                #rospy.logwarn('before isPtIntoCostMap')
                if self.isPtIntoCostMap(targetPose.position.x,targetPose.position.y):
                    return False
                return True
        except Exception as e:
            rospy.logwarn("{class_name} : Service make plan call failed: %s".format(class_name=self.__class__.__name__) % e)
            rospy.logwarn("{class_name} : Service make plan call failed  target goal: %s".format(class_name=self.__class__.__name__),str(targetPose.position))
            rospy.logwarn("{class_name} : Service make plan call failed  robot pose: %s".format(class_name=self.__class__.__name__),str(start.pose.position))
            return False





    
            
