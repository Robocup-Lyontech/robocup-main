__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod

class AbstractFaceScore:
    __metaclass__ = ABCMeta

    @abstractmethod
    def face_score(self, people, tracked_people): pass


    @abstractmethod
    def update_face_score_content(self, tracked_people_id_to_remove_list): pass

    @abstractmethod
    def add_face_to_tracked(self, people, tracked_people): pass