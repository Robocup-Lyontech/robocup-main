__author__ = 'Jacques Saraydaryan'

import time
import Queue

class ColorCandidate:
    FIFO_QUEUE_SIZE = 20

    def __init__(self):
        self.fifoColor = Queue.Queue(maxsize=self.FIFO_QUEUE_SIZE)
        self.sum_hsv=[0,0,0]
        self.last_update_time = time.time()

    def add_color(self, hsv):
        if self.fifoColor.qsize() >= self.FIFO_QUEUE_SIZE:
            out_hsv = self.fifoColor.get(timeout=0.5)
            self.sum_hsv[0] = self.sum_hsv[0] - out_hsv[0]
            self.sum_hsv[1] = self.sum_hsv[1] - out_hsv[1]
            self.sum_hsv[2] = self.sum_hsv[2] - out_hsv[2]

        self.fifoColor.put(hsv,timeout=0.5)
        self.sum_hsv[0] = self.sum_hsv[0] + hsv[0]
        self.sum_hsv[1] = self.sum_hsv[1] + hsv[1]
        self.sum_hsv[2] = self.sum_hsv[2] + hsv[2]
        self.last_update_time = time.time()


    def get_avg_color(self):
        avg_hsv=[0,0,0]

        if self.fifoColor.qsize()>0:
            avg_hsv[0] = self.sum_hsv[0] / float(self.fifoColor.qsize())
            avg_hsv[1] = self.sum_hsv[1] / float(self.fifoColor.qsize())
            avg_hsv[2] = self.sum_hsv[2] / float(self.fifoColor.qsize())

        return avg_hsv