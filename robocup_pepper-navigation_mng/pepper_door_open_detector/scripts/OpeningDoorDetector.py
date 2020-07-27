#!/usr/bin/env python  
import qi
import sys
import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
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
		rospy.Subscriber('/laser', LaserScan, self.laserCallback, queue_size=1)
		rospy.spin()

	def laserCallback(self,data):
		total_len=len(data.ranges)
		center_index=int(round((total_len/2.0) -1))
		# get values around the center
		rospy.logdebug( "length: "+ str(total_len)+", index:"+str(center_index))
		minValue_tmp=5000
		minIndex_tmp=-1
		
		for i in range(0,self.values_around+1):
			rospy.logdebug( "i:"+str(i)+",c+i:"+str(center_index+i)+",c-i:"+str(center_index-i))
			if minValue_tmp > data.ranges[center_index+i]:
				minIndex_tmp=center_index+i
				minValue_tmp=data.ranges[center_index+i]
			if minValue_tmp > data.ranges[center_index-i]:
				minIndex_tmp=center_index+i
				minValue_tmp=data.ranges[center_index-i]
		rospy.loginfo( "min distance:"+str(minValue_tmp)+", index laser:"+str(minIndex_tmp))
		self.minValue=minValue_tmp
		rospy.logdebug( "min distance global:"+str(self.minValue))
		self.minIndex=minIndex_tmp

		if self.minValue > self.min_distance:
			self.publisher.publish("START")

	def minFrontValueSrvCallback(self,req):
			result=self.minValue
			return {'value':result}

if __name__=="__main__":
	minFVD=MinFrontValueDetector()

