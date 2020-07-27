__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import random
import rospy
from threading import Timer

class AbstractNavStrategy:
    _retry_max_nb = 3
    _maxTimeElapsed = 60*10 # in second
    _timeout_checker=False
    _actMove_base=''
    _maxWaitTimePerGoal=60*2
    _retry_nb=0
    _t_timer=''

    def __init__(self,actMove_base):
        self._actMove_base=actMove_base
        pass


    @abstractmethod
    def goto(self, sourcePose, targetPose): pass

    @abstractmethod
    def stopAll(self): pass

    def startTimeWatch(self):
        timeout=self._maxTimeElapsed
        self.startTimeWatchWithTimeOut(self._maxTimeElapsed)

    def startTimeWatchWithTimeOut(self,tOut):
        timeout=tOut
        self._t_timer = Timer(timeout, self._timeout_checker)
        self._t_timer.start()
    
    def reset(self):
        self._retry_nb=0
        self._timeout_checker=False
        try:
            self._t_timer.cancel()
        except Exception as e:
            rospy.loginfo("Unable to reset timer: %s" % e)

    def setMaxNbRetry(self,maxNbRetry):
        self._retry_max_nb=maxNbRetry

    def setMaxTimeElapsed(self,maxElapsedTime):
        self._maxTimeElapsed=maxElapsedTime

    def setMaxTimeElapsedPerGoal(self,maxElapsedTimePerGoal):
        self._maxWaitTimePerGoal=maxElapsedTimePerGoal



