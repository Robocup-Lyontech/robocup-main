__author__ = 'Jacques Saraydaryan'

import time

class FaceCandidate:

    def __init__(self):
        self.label="None"
        self.weight=0
        self.max_score=0
        self.last_update_time = time.time()

    def inc_weight(self,value):
        """
        Update the current weight AND the last update time
        :param value: value of the increment
        :return: noting
        """
        self.weight = self.weight + value
        self.last_update_time = time.time()