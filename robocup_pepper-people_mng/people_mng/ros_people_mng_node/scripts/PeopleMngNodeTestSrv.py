#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ros_people_mng_srvs.srv import ProcessPeopleFromImg

def LoadImgAndUseSrv():
    rospy.init_node('detect_people_meta_info_test', anonymous=False)
    _bridge = CvBridge()
    test_folder=rospy.get_param('imgtest_folder','../data')

    #Load Image
    #img_loaded1 = cv2.imread(test_folder+'/group-in-line.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/group-diff-position.jpg')
    img_loaded1 = cv2.imread(test_folder+'/couple.jpg')

    
    msg_im1 = _bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")   
    
    #call service to detect color
    rospy.wait_for_service('detect_people_meta_srv')
    try:
        process_people_from_img_srv = rospy.ServiceProxy('detect_people_meta_srv', ProcessPeopleFromImg)
        content1 = process_people_from_img_srv(msg_im1)
        rospy.loginfo(content1)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s",e)

    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        LoadImgAndUseSrv()
    except rospy.ROSInterruptException:
        pass