#!/usr/bin/env python
import qi
import sys
import time
import numpy as np
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from pepper_door_open_detector.srv import MinFrontValue

class MinFrontValueDetector():

	publisher=''
	min_distance=0.8
	values_around=3
	minValue=5000
	minIndex=-1

	def __init__(self):
		self.configure()

	def configure(self):
		rospy.init_node('pepper_open_door_detector')
		self.min_distance=rospy.get_param('min_distance',0.8)
		rospy.logdebug("Param: min_distance:"+str(self.min_distance))

		#declare ros service
		self.minFrontValueSrv = rospy.Service('min_front_value_srv', MinFrontValue, self.minFrontValueSrvCallback)

		self.publisher=rospy.Publisher("/start", String, queue_size=1)
		rospy.Subscriber('/sonar', Range, self.sonarCallback, queue_size=1)
		rospy.spin()

	def sonarCallback(self,data):
		self.minValue=data.range
		rospy.logdebug("SONAR RANGE:"+str(data.range)+", distance_min before start:"+str(self.min_distance))
		if self.minValue < self.min_distance:
			self.publisher.publish("START")

	def minFrontValueSrvCallback(self,req):
		result=self.minValue
		return {'value':result}

if __name__=="__main__":
	minFVD=MinFrontValueDetector()
