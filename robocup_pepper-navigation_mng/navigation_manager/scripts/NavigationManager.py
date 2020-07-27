#!/usr/bin/env python
__author__ = 'Kathrin Evers'
__author__ ='Jacques Saraydaryan'

# ###############################
#       Import for ROS         #
################################
import rospy
import uuid
import tf
import math
import json
from tf import TransformListener
from robocup_msgs.msg import InterestPoint, Action, NavigationAction
from robocup_msgs.msg import gm_bus_msg
#from nav_pos_gateway_client.msg import NavGoalToGateway
from map_manager.srv import getitP_service
#from follow_me_mng.msg import FollowMeCmd
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation_manager.msg import NavMngAction
# Brings in the SimpleActionClient
import actionlib

from common.navstrategy.SimplyGoNavStrategy import SimplyGoNavStrategy
from common.navstrategy.GoAndRetryNavStrategy import GoAndRetryNavStrategy
from common.navstrategy.GoCleanRetryNavStrategy import GoCleanRetryNavStrategy
from common.navstrategy.GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy
from common.navstrategy.GoCRRCloseToGoal import GoCRRCloseToGoal



######### Command to Test
##  rostopic pub /gm_bus_command robocup_msgs/gm_bus_msg "{'action': 'NP', 'action_id': '1', 'payload': 'A_sim', 'result': 0}" 
## rostopic pub /gm_bus_command robocup_msgs/gm_bus_msg "{'action': 'NT', 'action_id': '1', 'payload': 'A_sim', 'result': 0}" 
#########

class Nm:
    # Initialize global variables
    _gm_bus_pub = ""
    _gm_bus_sub = ""
    _getPoint_service = ""
    _pub_goal = ""
    _sub_goal = ""
    #_pub_follow_me_order = ""
    #_sub_follow_me_answer = ""
    _tflistener = ""
    _current_itP = ""
    _current_action_id = ""
    _current_action_result = 0
    _current_payload = ""
    _current_message_id = 0
    _current_message_action = ""
    _current_gateway_id = ""
    _repetitions = 0
    #_current_follow_id = 0
    #_last_body_time = 0
    #_current_body_id = ""
    _actMove_base=""
    _actionToServiceMap={}
    _navigationStrategyMap={}

    def __init__(self):
        self.configure()

        #
        #  Set the initial configuration of the current Node
        #

    def configure(self):
        rospy.init_node('navigation_management_server')

        #initiate function on action
        self._actionToServiceMap={"NP":self.npAction,"Goto":self.gotoAction,"NF":self.nfAction,"NFS":self.nfsAction,"NT":self.ntAction}
        
        # initialize services and topics as well as function calls
        self._gm_bus_pub = rospy.Publisher("gm_bus_answer", gm_bus_msg, queue_size=1)
        self._gm_bus_sub = rospy.Subscriber("gm_bus_command", gm_bus_msg, self.gmBusListener)
        #FIXME wait the service ?
        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)

        # Subscribe to the move_base action server
        self._actMove_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        #FIXME What happen if action server is not available ?
        finished = self._actMove_base.wait_for_server(rospy.Duration(10))
        if finished:
            rospy.loginfo("Connected to move base server")
        else:
            rospy.logwarn("Couldn't connect to move base server")

        self._navigationStrategyMaps={"Simple":SimplyGoNavStrategy(self._actMove_base),"Retry":GoAndRetryNavStrategy(self._actMove_base),"CleanAndRetry":GoCleanRetryNavStrategy(self._actMove_base),"CleanRetryReplay":GoCleanRetryReplayLastNavStrategy(self._actMove_base),"CRRCloseToGoal":GoCRRCloseToGoal(self._actMove_base)}
        self._tflistener = TransformListener()

        self._actionServer = actionlib.SimpleActionServer('navigation_manager', NavMngAction, self.executeActionServer, False)
        self._actionServer.start()


        rospy.spin()  # spin() simply keeps python from exiting until this node is stopped

    def gmBusListener(self, data):
        if data.action in self._actionToServiceMap.keys():
            try:
                #call the process associating to the action
                self._actionToServiceMap[data.action](data)
            except Exception as e:
                rospy.logwarn("unable to find or launch function corresponding to the action:, error:[%s]:%s",str(data.action), str(e))


    def executeActionServer(self, goal):
        isActionSucceed=False
        try:

            # call the process associating to the action
            #isActionSucceed=self._actionToServiceMap[data.action](data)
            ### FIXME need to rework all _actionToServiceMap call...
            if goal.action == "NP" :
                current_navigationStrategy=self._navigationStrategyMaps[goal.navstrategy]  
                isActionSucceed=self.navigateToGoal("None",goal.itP,goal.action,current_navigationStrategy,goal.itP_point.x,goal.itP_point.y)
            elif goal.action == "NT":
                self.turnAround(float(goal.rotation_angle))
                isActionSucceed=True
    
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded()
        else:
            self._actionServer.set_aborted()
        return

    #
    # navigate to point
    #
    def navigate(self,current_message_id,current_itP,current_message_action,navigationStrategy):
        self.navigateToGoal(current_message_id,current_itP,current_message_action,navigationStrategy,0,0)

    def navigateToGoal(self,current_message_id,current_itP,current_message_action,navigationStrategy,x,y):
        if current_itP == '':
            pt=Pose()
            pt.position.x=x
            pt.position.y=y
            pt.orientation.x=0
            pt.orientation.y=0
            pt.orientation.z=0
            pt.orientation.w=1
            # Step 2: Use a navigation strategy
            result= navigationStrategy.goto(None,pt)
        else:
            # Step 1: get pose of sent interest point label
            itPoint = self._getPoint_service(current_itP)
            # Step 2: Use a navigation strategy
            result= navigationStrategy.goto(None,itPoint.itP.pose)
        
        # Step 3: Send result to the general Manager
        resultId=4 #Failure
        if result:
            resultId=3 #Success

        gm_result = gm_bus_msg()
        gm_result.action = current_message_action
        gm_result.action_id = current_message_id
        gm_result.payload = ""
        gm_result.result = resultId
        self._gm_bus_pub.publish(gm_result)
        return result


    #
    # navigate by following a person
    #
    def follow(self):
        ## publish a follow me order in the corresponding topic
        #self._current_follow_id = str(uuid.uuid1())
        #follow_order = FollowMeCmd()
        #follow_order.safe_zone_radius = 0
        #follow_order.id_people = "ALL"
        #follow_order.follow = True
        #follow_order.id = ""
        #follow_order.result = -1
        ##self._pub_follow_me_order.publish(follow_order)
        pass

    def followStop(self):
        ## publish a follow me order in the corresponding topic
        #follow_order = FollowMeCmd()
        #follow_order.safe_zone_radius = 0
        #follow_order.id_people = self._current_payload
        #follow_order.follow = False
        #follow_order.id = self._current_follow_id
        #follow_order.result = -1
        ##self._pub_follow_me_order.publish(follow_order)
        pass

    def followAnswer(self, data):
        ## answer of the following
        #if data.id == self._current_follow_id:
        #    if data.result == 4:  # failure
        #        # FIXME: alternative procedure if following didn't succeed, e.g. go to exit door, announce the loss
        #        print "I lost my operator :("
        #    elif data.result == 3:  # success
        #        # FIXME: Does this ever happen?
        #        print "I successfully followed my operator"
        pass

    #
    # turn around
    #
    def turnAround(self, data):

        # 1: load current orientation
        rospy.loginfo("In turn around function")
        now = rospy.Time(0)
        self._tflistener.waitForTransform("/map", "base_footprint", now, rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "base_footprint", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]

        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2] + data
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        robotPose.orientation.x = q[0]
        robotPose.orientation.y = q[1]
        robotPose.orientation.z = q[2]
        robotPose.orientation.w = q[3]

        #FIXME TO BE TESTED
        #strategy to apply
        current_navigationStrategy=self._navigationStrategyMaps["CRRCloseToGoal"]
        #Use a navigation strategy
        rospy.loginfo(robotPose)
        result= current_navigationStrategy.goto(None,robotPose)
        return result
        
        #self._current_gateway_id = str(uuid.uuid1())
        #goal_message = NavGoalToGateway()
        #goal_message.uuid = self._current_gateway_id
        #goal_message.pose = robotPose
        #goal_message.result = -1
        #self._pub_goal.publish(goal_message)

        # gm_result = gm_bus_msg()
        # gm_result.action = self._current_message_action
        # gm_result.action_id = self._current_message_id
        # gm_result.payload = ""
        # gm_result.result = 3
        # self._gm_bus_pub.publish(gm_result)
    def npAction(self,data):
        # navigate to point
        self._current_itP = data.payload
        rospy.loginfo("navigation: interest point = %s", self._current_itP)
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self._repetitions = 0
        #strategy to apply
        #current_navigationStrategy=self._navigationStrategyMaps["Simple"]
        #current_navigationStrategy=self._navigationStrategyMaps["CleanAndRetry"]  
        current_navigationStrategy=self._navigationStrategyMaps["CleanRetryReplay"]  
             
        self.navigate(self._current_message_id,self._current_itP,self._current_message_action,current_navigationStrategy)

    def gotoAction(self,data):
        # navigate to point
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        rad = float()
        try:
            rospy.loginfo("trying to execute the Goto action")
            payload = json.loads(data.payload)
            current_angle = payload["rotation"]
            rospy.loginfo("received this angle: %s", current_angle)
            if float(current_angle) == 0:
                rad = 1
            else:
                rad = float(current_angle) / 180 * float(math.pi)
            rospy.loginfo("angle in rad: %f", rad)
        except Exception as e:
            rospy.logwarn("unable to load payload content [%s]", str(e))
        self.turnAround(rad)

    def nfAction(self,data):
        # navigate while following a human
        # data.payload
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self._current_payload= data.payload
        self.follow()
        
    def nfsAction(self,data):
        # stop navigating while following a human
        # data.payload
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self.followStop()
         
    def ntAction(self,data):
        # turn around (180 degrees)
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self.turnAround(float(math.pi))


if __name__ == '__main__':
    nm = Nm()
