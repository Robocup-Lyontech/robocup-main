#!/usr/bin/env python  
import qi
import sys
import time
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo


def main(msg):	
	global motion

	motion.setStiffnesses("Head",1.0)
	motion.setAngles("Head",[0.,0.],0.05)
	#print motion.getAngles("Head",False)
	time.sleep(2)


if __name__=="__main__":
	rospy.init_node('pepper_head_pose_fix')
   	from optparse import OptionParser
   	parser = OptionParser()
   	#parser.add_option("--ppointcloud", dest="ppointcloud", default=True)
   	#parser.add_option("--plaser", dest="plaser", default=False)
   	parser.add_option("--pip", dest="pip", default="127.0.0.1")
   	parser.add_option("--pport", dest="pport", default=9559)
	(options, args) = parser.parse_args()
	
	ip =  options.pip
	port = options.pport
	session = qi.Session()

	try:
		session.connect("tcp://" + str(ip) + ":" + str(port))
	except RuntimeError:
		print ("Connection Error!")
		sys.exit(1)

	motion =  session.service("ALMotion")
	autolife = session.service("ALAutonomousLife")
	if autolife.getState() is not 'disabled':
		autolife.setState('disabled')
	
	posture = session.service("ALRobotPosture")
	if posture.getPostureFamily() is not "Stand":
		posture.goToPosture("Stand",0.3)

	rospy.Subscriber('/pepper_robot/camera/depth/camera_info', CameraInfo, main, queue_size=1)
	rospy.spin()

