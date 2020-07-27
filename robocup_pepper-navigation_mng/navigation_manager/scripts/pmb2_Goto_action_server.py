#!/usr/bin/env python

import rospy

import actionlib

import actionlib_tutorials.msg

import pmb2_nav_action.msg

class GoToPositionAction(object):
    # create messages that are used to publish feedback/result
    #_feedback = actionlib_tutorials.msg.FibonacciFeedback()
    #_result = actionlib_tutorials.msg.FibonacciResult()

    _feedback = pmb2_nav_action.msg.GoToInterestPointFeedback()
    _result = pmb2_nav_action.msg.GoToInterestPointResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pmb2_nav_action.msg.GoToInterestPointAction, execute_cb=self.execute_cb, auto_start= False)
        #self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("I'm in the __init__")
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        rospy.loginfo("Recieved goal from the client : " + str(goal))
        # append the seeds for the fibonacci sequence
        #self._feedback.sequence = []
        #self._feedback.sequence.append(0)
        #self._feedback.sequence.append(1)

        # publish info to the console for the user
        #rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        rospy.loginfo('%s: Executing...' % (self._action_name))
        #rospy.loginfo("I'm enterring the execute_cb function")

        '''
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        '''
        # Action execution test
        cpt = 0
        while(cpt<20):
            rospy.loginfo('Doing..')
            #self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
            cpt += 1
            # preempt request
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False

        '''
        # preempt request
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        '''

        if success:
            # self._result.sequence = self._feedback.sequence
            self._result.result = self._feedback.feedback
            self._result.result = 1
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('GoToPose')
    server = GoToPositionAction(rospy.get_name())
    # server = GoToPositionAction("ninja")
    rospy.spin()
