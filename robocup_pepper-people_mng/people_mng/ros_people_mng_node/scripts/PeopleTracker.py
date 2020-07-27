#!/usr/bin/env python  
__author__ = 'Jacques Saraydaryan'

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ros_people_mng_msgs.msg import PeopleMetaInfoDetails,PeopleMetaInfo,PeopleMetaInfoList
from process.PeopleMetaTrackerMng import PeopleMetaTrackerMng
from process.DisplayMetaData import DisplayMetaData
from visualization_msgs.msg import MarkerArray


class PeopleMngNode:

    def __init__(self):
        rospy.init_node('people_mng_tracker', anonymous=False)
        data_folder = rospy.get_param('imgtest_folder', '../data')
        self.camera_frame_id = rospy.get_param('camera_frame_id', 'camera_frame')
        stat_folder = rospy.get_param('stat_folder', '/tmp/')


        self._bridge = CvBridge()
        self.tracker = PeopleMetaTrackerMng(stat_folder)
        self.display=DisplayMetaData(data_folder+"/icon/",False,True,False)

        # Subscribe to the image 
        self.sub_rgb = rospy.Subscriber("/people_meta_info", PeopleMetaInfoList, self.detect_people_meta_callback, queue_size=1)
        self.pub_people_meta_info = rospy.Publisher("/tracked_people_meta_info", PeopleMetaInfoList, queue_size=1)
        self.pub_people_meta_info_img = rospy.Publisher("/tracked_people_meta_info_img", Image, queue_size=1)
        self.pub_tracked_people_marker = rospy.Publisher("/tracked_people_meta_info_marker", MarkerArray, queue_size=1)
        rospy.spin()
        rospy.loginfo("Exiting tracker...")
        self.tracker.statMng.save_stat()
        self.tracker.stop_forgetting_function()


    def detect_people_meta_callback(self,req):
        #rospy.loginfo("--------------------> Get data into tracker")
        #tracked_people_list=self.tracker.track_people(req.peopleList)
        tracked_people_list = self.tracker.track_people_best_per_tracked(req.peopleList)


        marker_array=self.display.displayTracker3DMarker(tracked_people_list, self.camera_frame_id)
        self.pub_tracked_people_marker.publish(marker_array)

        #req.img --> image to display meta info
        cv_image = self._bridge.imgmsg_to_cv2(req.img, desired_encoding="bgr8")
        #cv_img_people = self.display.displayResult(req, cv_image)
        cv_img_people_tracker = self.display.displayTrackerResult(tracked_people_list, cv_image)
        msg_img = self._bridge.cv2_to_imgmsg(cv_img_people_tracker, encoding="bgr8")
        # publish image with MetaData
        self.pub_people_meta_info_img.publish(msg_img)





def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()