__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod

class AbstractPoseScore:
    __metaclass__ = ABCMeta

    @abstractmethod
    def pose_score(self, people, tracked_people, new_pose): pass


    @abstractmethod
    def update_pose_score_content(self): pass