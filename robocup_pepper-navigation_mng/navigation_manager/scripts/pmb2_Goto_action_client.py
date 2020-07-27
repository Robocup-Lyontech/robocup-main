#! /usr/bin/env python

from __future__ import print_function
import rospy


# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
# import actionlib_tutorials.msg
import pmb2_nav_action.msg
import geometry_msgs


def GoToPositionActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    # client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
    client = actionlib.SimpleActionClient('GoToPose', pmb2_nav_action.msg.GoToInterestPointAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    #goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
    selected_pose = geometry_msgs.msg.PoseStamped()
    
    # Entrance pose
    selected_pose.header.frame_id = "map"
    selected_pose.pose.position.x = -3.09927248955
    selected_pose.pose.position.y = 0.147106766701
    selected_pose.pose.orientation.z = 0.965981851705
    selected_pose.pose.orientation.w = 0.25860986481


    goal = pmb2_nav_action.msg.GoToInterestPointGoal(selected_pose)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pmb2_GoTo_action_client')
        result = GoToPositionActionClient()
        print("Result: " + str(result.result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)