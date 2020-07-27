#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from naoqi import ALProxy
import qi
import argparse
import sys
import time
import rospy

class moveSound:

	def __init__(self,session,ip,port):
		rospy.loginfo("Connecting to"+str(ip)+"with port"+str(port))
		self.session=session
		# Get the service ALAutonomousLife, ALTracker, ALMotion and ALRobotPosture.
		self.allMove= ALProxy("ALAutonomousLife",ip,port)
		self.motion= ALProxy("ALMotion", ip, port)
		self.posture= ALProxy("ALRobotPosture", ip, port)
		self.tracker= ALProxy("ALTracker", ip, port)

	def start(self):

		#Désactiver le mode live.
		#allMove.setState("disabled")

		# Wake up.
		self.motion.wakeUp()

		# Go to posture stand
		fractionMaxSpeed = 0.8
		self.posture.goToPosture("StandInit", fractionMaxSpeed)

		#Enable Whole Body Balancer and Move.
		self.motion.wbEnable(True)
		self.motion.moveInit()

		#Détecte le son et se déplace face à l'interlocuteur
		# => Ajouter la cible à suivre (son) => target.registerTarget(targetName, [distance, confidence])
		targetSound= "Sound"
		self.tracker.registerTarget(targetSound,[0.8,0.5])

		#Set mode => type de déplacement lorsque la cible est reçue
		# "Move" = déplacement des pieds ; #"Head" = déplacement de la tête ; "WholeBody" = déplacement du corps (pieds fixés)
		mode= "Move"
		self.tracker.setMode(mode)

		# Then, start tracker.	
		self.tracker.track(targetSound)

		
		rospy.loginfo( "ALTracker successfully started, now show your face  to robot and speak!")
		rospy.loginfo( "Use Ctrl+c to stop this script.")


	def stop(self):
		# Stop tracker.
		self.tracker.stopTracker()
		self.tracker.unregisterAllTargets()
		# Go to rest position
		self.motion.wbEnable(False)
		self.motion.rest()
		rospy.loginfo( "ALTracker stopped.")

##if __name__ == "__main__":
#	parser = argparse.ArgumentParser()
#	parser.add_argument("--ip", type=str, default="192.168.0.55",help="Robot IP address. On robot or Local Naoqi: use '134.214.50.49'.")
#	parser.add_argument("--port", type=int, default=9559,help="Naoqi port number")
#
#	args = parser.parse_args()
#	robot_session = qi.Session()
#
#	try:
#		robot_session.connect("tcp://" + args.ip + ":" + str(args.port))
#	except RuntimeError:
#		print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"+"Please check your script arguments. Run with -h option for help.")
#		sys.exit(1)
#
#	moveSound(robot_session,args.ip, args.port)
