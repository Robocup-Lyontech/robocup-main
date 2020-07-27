#!/usr/bin/env python

import rospy
from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTSimulation import LTSimulation
from tf import TransformListener
from geometry_msgs.msg import PointStamped

from meta_lib.LTSimulation import LTSimulation
import os 
import math
import numpy as np
from ros_people_mng_msgs.msg import PeopleMetaInfoDetails, PeopleMetaInfo, PeopleMetaInfoList

class Test:

    def __init__(self):

        rospy.init_node("test_people")
        self.listener = TransformListener()

        # self._lt_perception = LTPerception()
        # self._lt_simulation = LTSimulation()
        # rospy.loginfo("{class_name}: SETTING UP GUESTS FOR SIMULATED SCENARIO".format(class_name=self.__class__.__name__))
        # self._lt_simulation.reset_guests_for_receptionist()
        # self._lt_simulation.guest_spawner_for_receptionist("G1_entrance")
        # self._lt_simulation = LTSimulation()

        # self._lt_simulation.reset_guests_for_receptionist()
        # self._lt_simulation.guest_spawner_for_receptionist("G1_entrance")

        # self.authorize_sub = False

        # self._lt_navigation = LTNavigation()

        self.sub_people_meta_info = rospy.Subscriber("/people_meta_info", PeopleMetaInfoList, self.handle_callback)

        # self.authorize_sub = True
        # rotation_angle = 2*math.pi
        # response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)

    # def handle_callback(self,req):
    #     liste = req.peopleList
    #     rospy.logwarn("HEADER : %s",str(req.img.header))
    #     for item in liste:
    #         pose = item.pose
    #         rospy.logwarn("PEOPLE : %s POSE x: %s y: %s",item.label_id,pose.position.x,pose.position.y)
            
    #         now = rospy.Time(0)
    #         object_point = PointStamped()
    #         object_point.header.frame_id = "palbator_arm_kinect_link"
    #         object_point.header.stamp = now
    #         object_point.point.x = pose.position.x
    #         object_point.point.y = pose.position.y
    #         object_point.point.z = pose.position.z
    #         rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
    #         self.listener.waitForTransform("/map", "/palbator_arm_kinect_link", now, rospy.Duration(20))
    #         target = self.listener.transformPoint("/map",object_point)
    #         rospy.loginfo("{class_name} : Object coords in map : %s".format(class_name=self.__class__.__name__),str(target))

    #     rospy.logerr("----")
        # rospy.loginfo(req.img)


        # itp_name = "place_entrance_recep"
        # self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal",itp_name , 90.0)

        # listener = TransformListener()
        
        # response = self._lt_perception.learn_people_meta_from_img_topic(u"Bill",10)

        # rospy.sleep(3)

        # response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/fakePkgForTabletPalbator/tablet_code/robocup_palbator-hri_js/public/img/people/John.png","John",10)

        # rospy.sleep(3)

        # response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/fakePkgForTabletPalbator/tablet_code/robocup_palbator-hri_js/public/img/people/John_simu.png","Robert",10)

        # response = self._lt_perception.reset_people_meta_info_map()



        # rotation_angle = math.pi
        # response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)

        # self.detected_bill = False

        # rotation_angle = math.pi/4.0
        # while not self.detected_bill:
        #     response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)
        # response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)
        # result = response.payload
        # rospy.logwarn("RESULT : %s",str(result))
        # if result != None and result != {}:
        #     detection = result.peopleMetaList.peopleList
        #     rospy.logwarn("DETECTION : %s",str(detection))
        #         for people in detection:
        #             if people.label_id == "Bill":
        #                 self.detected_bill = True
        #                 rospy.logerr("fbhzebunrvineriob,eirb,ioerb,roe,bioer,bpoer;plgpkerojgeorgkoerkgoerkgoerger")
        #                 break



        
        # itp_name = "test_People_John"
        # self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal",itp_name , 90.0)
        # self.path_folder_to_save_imgs = "../../../../fakePkgForTabletPalbator/tablet_code/robocup_palbator-hri_js/public"
        
        # dir_path = os.path.dirname(os.path.realpath(__file__))
        # rospy.logwarn("DIR PATH : %s",dir_path)
        # img_path = os.path.join(dir_path,self.path_folder_to_save_imgs)
        # img_path = os.path.join(img_path,"img/people/G1_test.png")
        # self._lt_perception.take_picture_and_save_it_Palbator(img_path)
        
        # response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)

        # rospy.logwarn("RESPONSE : %s",str(response.payload))

        # payload = response.payload

        # if payload == {}:
        #     rospy.logerr("NADAAAAA")
        # else:
        #     detection = response.payload.peopleMetaList.peopleList
        #     rospy.loginfo("DETECTION : %s",str(detection))

        # pose = response.payload.peopleMetaList.peopleList[0].pose

        
        # now = rospy.Time(0)
        # object_point = PointStamped()
        # object_point.header.frame_id = "palbator_arm_kinect_link"
        # object_point.header.stamp = now
        # # object_point.point.x = pose.position.x
        # # object_point.point.y = pose.position.y
        # # object_point.point.z = pose.position.z
        # object_point.point.x = 2.7059
        # object_point.point.y = -0.8693
        # object_point.point.z = 0.0

        # rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
        # listener.waitForTransform("/base_footprint", "/palbator_arm_kinect_link", now, rospy.Duration(20))
        # target = listener.transformPoint("/base_footprint",object_point)

        # rospy.loginfo("{class_name} : Object coords in base_footprint : %s".format(class_name=self.__class__.__name__),str(target))

        # alpha = np.arctan(target.point.y/target.point.x)

        # rospy.logerr("ALPHA : %s",str(alpha))
        # rospy.logerr("ALPHA DEGRES : %s",str((alpha*360)/(2*math.pi)))
        # response_nav = self._lt_navigation.send_nav_rotation_order("NT", alpha , 90.0) 

        # self._bridge = CvBridge()
        # self.sub = rospy.Subscriber("/people_meta_info",PeopleMetaInfoList,self.handle_detection)

        # response = self._lt_perception.learn_people_meta_from_img_topic(name="Thomas",timeout=10)
        # response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg","Robert",10)

        # rospy.sleep(5)

        # response = self._lt_perception.get_people_name_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg",10)
        # rospy.loginfo("RESPONSE : %s",str(response.payload.peopleMetaList.peopleList))
        # rospy.logwarn("RESPONSE MSG : %s",str(response.msg))

        # response = self._lt_perception.take_picture_and_save_it_Palbator("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-general_mng/general_mng/scripts/test.png")

        # rospy.loginfo(response)

        # img_cv2 = self._bridge.imgmsg_to_cv2(response.payload.peopleMetaList.img, "bgr8")
        # cv2.imshow("test",img_cv2)

        # response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)
        # rospy.loginfo("RESPONSE : %s",str(response.payload))




if __name__ == "__main__":
    a = Test()
    # rotation_angle = 2*math.pi
    # response_nav = a._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)
    while not rospy.is_shutdown():
        rospy.spin()

