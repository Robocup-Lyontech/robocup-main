#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy

from convert_2d_to_3d.srv import get3Dfrom2D
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes
from robocup_msgs.msg import Entity, Entity2D, EntityList
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Pose2D,Point


class darknet_2d_to_3d:
    DEFAULT_FILTER_VALUE="person"
    DEFAULT_SOURCE_FRAME_VALUE="CameraTop_optical_frame"
    DEFAULT_TARGET_FRAME_VALUE = "map"

    def __init__(self):

       
        self.filter=self.DEFAULT_FILTER_VALUE
        self.targetFrame=self.DEFAULT_TARGET_FRAME_VALUE
        self.source_frame=self.DEFAULT_SOURCE_FRAME_VALUE

        try:
            rospy.wait_for_service('/convert_2d_to_3d',5)
            rospy.loginfo("end service convert_2d_to_3d wait time")
            self._convert_2d_3d_service = rospy.ServiceProxy('convert_2d_to_3d', get3Dfrom2D)
        except Exception as e:
            rospy.logerr("Service convert_2d_to_3d call failed: %s" % e)

        self.sub=rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getDarkNetBoxesCallback)
        self.pub = rospy.Publisher('/objects/entity', EntityList, queue_size=1)

    def getDarkNetBoxesCallback(self,msg):
        entity_list=EntityList()
        list=[]

        for box in msg.bounding_boxes:
            if box.Class == self.filter:
                current_entity=Entity()
                current_entity.label=box.Class

                rgb_w = int(( box.xmax-box.xmin)/2+box.xmin);
                rgb_h = int((box.ymax-box.ymin)/2+box.ymin);
                current_pose2d=Pose2D()
                current_pose2d.x=rgb_w
                current_pose2d.y=rgb_h
                rospy.loginfo(box)
                rospy.loginfo(current_pose2d)

                current_point=self._convert_2d_3d_service(pose=current_pose2d,frame_id=self.source_frame)

                current_entity.pose.position.x=current_point.point.x
                current_entity.pose.position.y = current_point.point.y
                current_entity.pose.position.z = current_point.point.z
                current_entity.header.frame_id = self.targetFrame
                list.append(current_entity)
        entity_list.entityList = list

        self.pub.publish(entity_list)

    

if __name__ == '__main__':
    rospy.init_node('darknet_2d_to_3d_node')
    d2d_to3d = darknet_2d_to_3d()

    rospy.spin()