#!/usr/bin/env python

import smach
import smach_ros
import roslib
import rospy
from threading import Timer

# define state Foo
class RobotState(smach.State):
    def __init__(self,timeout,retry_nb,name,outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.name=name
        self.timeout=timeout
        self.retry_nb=retry_nb
        t = Timer(self.timeout, self.timeout_checker)
        t.start()

    def execute(self, userdata):
        print "method to override"
        #rospy.logerr("method to override")

    def timeout_checker(self):
        print "TIME OUT REACHED: state:"
        #rospy.logerr("TIME OUT REACHED: state:"+self.name())
