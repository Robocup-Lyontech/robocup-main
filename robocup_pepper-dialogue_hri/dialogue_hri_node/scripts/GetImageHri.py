#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Get an image. Display it and save it using PIL."""

import rospy
import qi
import argparse
import sys
import time
import Image
from dialogue_hri_srvs.srv import TakePicture

class TakeImage:

    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

        rospy.loginfo("TakeImage: READY TO PROCESS ACTION")

    def configureNaoqi(self):
        self._session = qi.Session()
        rospy.loginfo("TakeImage: try connecting to robot...")
        try:
            self._session.connect("tcp://" + self._ip + ":" + str(self._port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + self._ip + "\" on port " + str(self._port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False

        self._video_service = self._session.service("ALVideoDevice")
        # http://doc.aldebaran.com/2-5/family/pepper_technical/video_2D_pep_v18a.html#cameraresolution-ov5640
        self._resolution = 4    # 2: VGA 3: 4*VGA 4:16*VGA
        self._colorSpace = 11   # RGB
        self._videoClient = self._video_service.subscribe("python_client", self._resolution, self._colorSpace, 5)
        self._t0 = time.time()
        rospy.loginfo("TAKEPICTURE: CONFIGURATION NAOQI OK")
        return True

    def configure(self):
        #create Service
        self._activateMoveSound_service = rospy.Service('take_picture_service', TakePicture, self.takeImg)
        rospy.loginfo("TAKEPICTURE: CONFIGURATION ACTION OK")

    def takeImg(self,req):

        naoImage = self._video_service.getImageRemote(self._videoClient)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        image_string = str(bytearray(array))


        try:
            # Create a PIL Image from our pixel array.
            #im = Image.fromstring("RGB", (imageWidth, imageHeight), image_string)
            im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)
            # Save the image.
            im.save(req.picture_full_path, "PNG")

            #Display the current given information very usefull
            #im.show()
        except Exception as e:

            print(e)

        return []

    def shutdown(self):
        self._video_service.unsubscribe(self._videoClient)


if __name__ == "__main__":
    rospy.init_node('pepper_move_sound_hri')
    ip=rospy.get_param('~ip',"192.168.0.189")
    port=rospy.get_param('~port',9559)

    takeImg=TakeImage(ip,port)
    rospy.spin()
    takeImg.shutdown()
