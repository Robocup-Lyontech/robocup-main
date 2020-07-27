#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from geometry_msgs.msg import Twist, Vector3


class CmdTwist(object):
    _cmdMsg = ''
    _duration = 0
    _startTime = 0  # in second
    _stopTime = 0  # in second

    def __init__(self, cmdMsg):
        self._cmdMsg = cmdMsg
        self._startTime = time.time()
    
    def setStopTime(self, stopTime):
        self._stopTime = stopTime
        self._duration = stopTime - self._startTime

    def duration(self):
       return self._duration

    def getTwistCmd(self):
        return self._cmdMsg

    def reverse(self):
        return Twist(
            linear=Vector3(x=self._cmdMsg.linear.x * -1, y=self._cmdMsg.linear.y * -1, z=self._cmdMsg.linear.z * -1),
            angular=Vector3(x=self._cmdMsg.angular.x * -1, y=self._cmdMsg.angular.y * -1, z=self._cmdMsg.angular.z * -1)
        )
