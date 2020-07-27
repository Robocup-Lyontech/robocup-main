__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy 
import cv2
import operator
import webcolors
from PersonMetaInfo import PersonMetaInfo
from visualization_msgs.msg import Marker, MarkerArray

class DisplayMetaData:
    PEOPLE_LABEL_UNKNOW="Unknwon"
    PEOPLE_LABEL_NONE="None"
    PEOPLE_STANDING="Standing"
    PEOPLE_SITTING="Sitting"
    PEOPLE_LYING="Lying"
    PEOPLE_HAND_CALL="Call"
    PEOPLE_HAND_LEFT_CALL="Left_Call"
    PEOPLE_HAND_RIGHT_CALL="Right_Call"
    PEOPLE_HAND_RIGHT_POINTING="Pointing Right"
    PEOPLE_HAND_LEFT_POINTING="Pointing Left"
    PEOPLE_HAND_CROSSED="Crossed"
    UNKNOWN="Undefined"

    ICON_STANDING="pStanding20b.png"
    ICON_SITTING="pSitting20b.png"
    ICON_LYING="pLying20b.png"
    ICON_HAND_LEFT_CALL="pHandLeftUpb.png"
    ICON_HAND_LEFT_POINTING="pHandLeftPointingb.png"
    ICON_HAND_RIGHT_CALL="pHandRightUpb.png"
    ICON_HAND_RIGHT_POINTING="pHandRightPointingb.png"
    ICON_HAND_CROSS="pHandCrossb.png"
    ICON_UNKNOWN="pUndefined20b.png"

    MESH_STANDING = "package://ros_people_mng_node/data/peopleStanding3.dae"
    MESH_SITTING = "package://ros_people_mng_node/data/peopleSitting2.dae"
    MESH_LYING = "package://ros_people_mng_node/data/peopleStanding2.dae"
    MESH_HAND_LEFT_CALL = "package://ros_people_mng_node/data/peopleCallLeft.dae"
    MESH_HAND_LEFT_POINTING = "package://ros_people_mng_node/data/peoplePointingLeft.dae"
    MESH_HAND_RIGHT_CALL = "package://ros_people_mng_node/data/peopleCallRight.dae"
    MESH_HAND_RIGHT_POINTING = "package://ros_people_mng_node/data/peoplePointingRight.dae"
    MESH_SITTING_HAND_LEFT_CALL = "package://ros_people_mng_node/data/peopleSittingcallLeft.dae"
    MESH_SITTING_HAND_LEFT_POINTING = "package://ros_people_mng_node/data/peopleSittingPointingLeft.dae"
    MESH_SITTING_HAND_RIGHT_CALL = "package://ros_people_mng_node/data/peopleSittingcallRight.dae"
    MESH_SITTING_HAND_RIGHT_POINTING = "package://ros_people_mng_node/data/peopleSittingPointingRight.dae"
    MESH_SITTING_HAND_CROSS = "package://ros_people_mng_node/data/peopleSittingCrossed.dae"
    MESH_HAND_CROSS = "package://ros_people_mng_node/data/peopleStandingCrossed.dae"
    MESH_UNKNOWN = "package://ros_people_mng_node/data/peopleStanding3.dae"

    COLOR_1=(0,255,0)
    COLOR_2=(0,0,255)

    def __init__(self,icon_folder,isProportionDisplayed,isColorRecDisplayer,isWaiting):
        self.iconMap={}
        self.meshMap={}
        self.icon_folder=icon_folder
        self.isProportionDisplayed=isProportionDisplayed
        self.isColorRecDisplayer=isColorRecDisplayer
        self.isWaiting=isWaiting
        self.marker_id_map={}
        self.marker_id_counter=0

        self.loadIcons()
        self.loadMesh()


    def loadIcons(self):
        self.iconMap[self.PEOPLE_STANDING]=cv2.imread(self.icon_folder+self.ICON_STANDING)
        self.iconMap[self.PEOPLE_SITTING]=cv2.imread(self.icon_folder+self.ICON_SITTING)
        self.iconMap[self.PEOPLE_LYING]=cv2.imread(self.icon_folder+self.ICON_LYING)
        self.iconMap[self.PEOPLE_HAND_LEFT_CALL]=cv2.imread(self.icon_folder+self.ICON_HAND_LEFT_CALL)
        self.iconMap[self.PEOPLE_HAND_LEFT_POINTING]=cv2.imread(self.icon_folder+self.ICON_HAND_LEFT_POINTING)
        self.iconMap[self.PEOPLE_HAND_RIGHT_CALL]=cv2.imread(self.icon_folder+self.ICON_HAND_RIGHT_CALL)
        self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING]=cv2.imread(self.icon_folder+self.ICON_HAND_RIGHT_POINTING)
        self.iconMap[self.PEOPLE_HAND_CROSSED]=cv2.imread(self.icon_folder+self.ICON_HAND_CROSS)
        self.iconMap[self.UNKNOWN]=cv2.imread(self.icon_folder+self.ICON_UNKNOWN)

    def loadMesh(self):
        self.meshMap[self.PEOPLE_STANDING]=self.MESH_STANDING
        self.meshMap[self.PEOPLE_SITTING]=self.MESH_SITTING
        self.meshMap[self.PEOPLE_LYING]=self.MESH_LYING
        self.meshMap[self.PEOPLE_HAND_LEFT_CALL]=self.MESH_HAND_RIGHT_CALL
        self.meshMap[self.PEOPLE_HAND_LEFT_POINTING]=self.MESH_HAND_RIGHT_POINTING
        self.meshMap[self.PEOPLE_HAND_RIGHT_CALL]=self.MESH_HAND_LEFT_CALL
        self.meshMap[self.PEOPLE_HAND_RIGHT_POINTING]=self.MESH_HAND_LEFT_POINTING
        self.meshMap[self.PEOPLE_HAND_CROSSED]=self.MESH_HAND_CROSS
        self.meshMap[self.UNKNOWN]=self.MESH_UNKNOWN

        self.meshMap[self.MESH_SITTING_HAND_LEFT_CALL] = self.MESH_SITTING_HAND_RIGHT_CALL
        self.meshMap[self.MESH_SITTING_HAND_LEFT_POINTING] = self.MESH_SITTING_HAND_RIGHT_POINTING
        self.meshMap[self.MESH_SITTING_HAND_RIGHT_CALL] = self.MESH_SITTING_HAND_LEFT_CALL
        self.meshMap[self.MESH_SITTING_HAND_RIGHT_POINTING] = self.MESH_SITTING_HAND_LEFT_POINTING
        self.meshMap[self.MESH_SITTING_HAND_CROSS] = self.MESH_SITTING_HAND_CROSS

    def displayResult(self,content_result,img):
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 600, 600)
        for people in content_result.peopleList:
            #img2=img.copy()
            if people.label_id!=self.PEOPLE_LABEL_UNKNOW and people.label_id!=self.PEOPLE_LABEL_NONE:
                self.createRec(img,people.details.boundingBox.points,self.COLOR_1,2)
            else:
                self.createRec(img,people.details.boundingBox.points,self.COLOR_2,2)
            
            if len(people.details.boundingBox.points)>=2:
                self.delta_y=int(round(people.details.boundingBox.points[0].y))
                self.displayIconPosture(img,people.details.boundingBox.points,people,20,0)
                self.displayIconHandLeftPosture(img,people.details.boundingBox.points,people,0,0)
                self.displayIconHandRightPosture(img,people.details.boundingBox.points,people,20,20)

            font = cv2.FONT_HERSHEY_DUPLEX
            self.displayColorRec(img,people.details.boundingBox.points,people.details.shirtColorList,20,0)
            cv2.putText(img, people.shirt_color_name, (int(round(people.details.boundingBox.points[1].x) + 22), self.delta_y-20 + 12),  font, 0.5, (0, 0, 0), 1)
            self.displayColorRec(img,people.details.boundingBox.points,people.details.trouserColorList,20,0)
            cv2.putText(img, people.trouser_color_name,(int(round(people.details.boundingBox.points[1].x) + 22), self.delta_y-20 + 12), font, 0.5, (0, 0, 0), 1)
            
            if  self.isColorRecDisplayer:
                if  self.isProportionDisplayed:
                    self.creatColorRecProporition(img,people.details.trouserRect.points,people.details.trouserColorList)
                self.createRec(img,people.details.trouserRect.points,(0,0,0),1)

            if  self.isColorRecDisplayer:
                if  self.isProportionDisplayed:
                    self.creatColorRecProporition(imgr,people.details.shirtRect.points,people.details.shirtColorList)
                self.createRec(img,people.details.shirtRect.points,(0,0,0),1)

            self.putTxtDistance(img,people.details.boundingBox.points,people,(255,255,255))
            
            self.putTxtIDLabel(img,people.details.boundingBox.points,people,(255,255,255))

           # cv2.imshow('image', img2)
           # cv2.waitKey(0)

        if self.isWaiting:
            cv2.namedWindow('image',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('image', 600,600)
            cv2.imshow('image',img)
            cv2.waitKey(0)

        return img

    def displayTrackerResult(self, tracked_people_list, img):
        for tracked_people in tracked_people_list:
            points=tracked_people.getBoundingBox(PersonMetaInfo.PERSON_RECT).points
            if tracked_people.label_id != self.PEOPLE_LABEL_UNKNOW and tracked_people.label_id != self.PEOPLE_LABEL_NONE:
                self.createRec(img, points, self.COLOR_1, 10)
            else:
                self.createRec(img, points, self.COLOR_2, 2)

            self.putTrackedPeopleTxt(img,points,tracked_people,(255,255,255))

        if self.isWaiting:
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('image', 600, 600)
            cv2.imshow('image', img)
            cv2.waitKey(0)

        return img


    def displayTracker3DMarker(self, tracked_people_list,frame_id):
        marker_array=MarkerArray()
        for tracked_people in tracked_people_list:
            marker_id=0
            try:
                marker_id= self.marker_id_map[tracked_people.id]
            except KeyError as e:
                self.marker_id_counter=self.marker_id_counter+1
                marker_id=self.marker_id_counter
                self.marker_id_map[tracked_people.id]=marker_id

            markerA=self.displayTrackerMarkerArrow(tracked_people,frame_id,marker_id)
            marker_array.markers.append(markerA)
            markerM=self.displayTrackerMarkerMesh(tracked_people,frame_id,marker_id+1000)
            marker_array.markers.append(markerM)

        return marker_array



    def displayTrackerMarkerArrow(self, tracked_people,frame_id,marker_id):
        marker=Marker()
        marker.header.frame_id=frame_id
        # marker.header.stamp=rospy.get_time()
        marker.ns = "tracker-people"
        marker.lifetime=rospy.Duration.from_sec(5)
        # marker.id = tracked_people.id
        # marker id must be integer
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose=tracked_people.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker


    def displayTrackerMarkerMesh(self, tracked_people,frame_id,marker_id):
        mesh_file=""
        if tracked_people.posture == self.PEOPLE_STANDING:
            mesh_file=self.meshMap[self.PEOPLE_STANDING]
            #rospy.logwarn("hand[0] :" + tracked_people.handPosture[0]+", hand[1]"+tracked_people.handPosture[1])
            if tracked_people.handPosture[1] == self.PEOPLE_HAND_CALL:
                mesh_file = self.meshMap[self.PEOPLE_HAND_LEFT_CALL]
            elif tracked_people.handPosture[1] == self.PEOPLE_HAND_LEFT_POINTING:
                mesh_file = self.meshMap[self.PEOPLE_HAND_LEFT_POINTING]
            if tracked_people.handPosture[0] == self.PEOPLE_HAND_CALL:
                mesh_file = self.meshMap[self.PEOPLE_HAND_RIGHT_CALL]
            elif tracked_people.handPosture[0] == self.PEOPLE_HAND_RIGHT_POINTING:
                mesh_file = self.meshMap[self.PEOPLE_HAND_RIGHT_POINTING]
            elif tracked_people.handPosture[0] == self.PEOPLE_HAND_CROSSED:
                mesh_file = self.meshMap[self.PEOPLE_HAND_CROSSED]

        elif tracked_people.posture == self.PEOPLE_SITTING:
            mesh_file = self.meshMap[self.PEOPLE_SITTING]
            if tracked_people.handPosture[1] == self.PEOPLE_HAND_CALL:
                mesh_file = self.meshMap[self.MESH_SITTING_HAND_LEFT_CALL]
            elif tracked_people.handPosture[1] == self.PEOPLE_HAND_LEFT_POINTING:
                mesh_file = self.meshMap[self.MESH_SITTING_HAND_LEFT_POINTING]
            if tracked_people.handPosture[0] == self.PEOPLE_HAND_CALL:
                mesh_file = self.meshMap[self.MESH_SITTING_HAND_RIGHT_CALL]
            elif tracked_people.handPosture[0] == self.PEOPLE_HAND_RIGHT_POINTING:
                mesh_file = self.meshMap[self.MESH_SITTING_HAND_RIGHT_POINTING]
            if tracked_people.handPosture[0] == self.MESH_HAND_CROSS:
                mesh_file = self.meshMap[self.MESH_HAND_CROSS]
        elif tracked_people.posture == self.PEOPLE_LYING:
            mesh_file = self.meshMap[self.PEOPLE_SITTING]


        marker=Marker()
        marker.header.frame_id=frame_id
        # marker.header.stamp=rospy.get_time()
        marker.ns = "tracker-people"
        # marker.id = tracked_people.id
        # marker id must be integer
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(5)
        marker.pose=tracked_people.pose
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        rgb = tracked_people.colorRGBMap[PersonMetaInfo.SHIRT_RECT]
        if len(rgb) >= 3:
            marker.color.r = round(rgb[0]/float(255),3)
            marker.color.g = round(rgb[1]/float(255),3)
            marker.color.b = round(rgb[2]/float(255),3)
        marker.mesh_resource = str(mesh_file)
        #rospy.logwarn("mesh file :"+str(mesh_file))
        return marker


    def createRec(self, img,points,(B,G,R),size):
        if len(points)>=2:
            cv2.rectangle(img, (int(round(points[0].x)), int(round(points[0].y))), (int(round(points[1].x)), int(round(points[1].y))), (B,G,R), size)

    def creatColorRecProporition(self,img,points,colorList):
        if len(points)>=2:
            y_start=int(round(points[0].y))
            total_y=int(round(points[1].y-points[0].y))
            size_to_fill=total_y
            mapColor={}
            for color in colorList:
                mapColor[color.percentage_of_img]=color.rgb

            sorted_mapColor=sorted(mapColor.items(),key=operator.itemgetter(0),reverse=False)
        
            for key,rgb in sorted_mapColor:
                y_size=int(round(total_y*key))
                size_to_fill=size_to_fill-y_size
                new_y=int(round(y_start+y_size))
                cv2.rectangle(img, (int(round(points[0].x)), int(round(y_start))), (int(round(points[1].x)), new_y), (rgb[2],rgb[1],rgb[0]), cv2.FILLED)
                y_start=y_start+y_size


    def getMaxColor(self,colorList):
        rgb=[]
        max=0.0
        for color in colorList:
            if color.percentage_of_img > max:
                max=color.percentage_of_img
                rgb=color.rgb
        return rgb


    def getMaxColorWebName(self,colorList):
        webColorname='white'
        max=0.0
        for color in colorList:
            if color.percentage_of_img > max:
                max=color.percentage_of_img
                webColorname=color.color_web
        return webColorname

    def displayColorRec(self,img,points,colorList,offset_y,delta_x):
        if len(points)>=2:
            delta=2
            x_offset=(int(round(points[1].x))+delta)+delta_x
            y_offset=self.delta_y
             #create little info about main shirt color
            (R,G,B)=webcolors.name_to_rgb(self.getMaxColorWebName(colorList))
            cv2.rectangle(img,( x_offset, y_offset),(x_offset+20, y_offset+20), (B,G,R), cv2.FILLED)
            self.delta_y=self.delta_y+offset_y
            





    def displayIconPosture(self,img,points,people,offset_y,delta_x):
        if len(points)>=2:
            delta=2
            x_offset=(int(round(points[1].x))+delta)+delta_x
            y_offset=self.delta_y             
            try:
                if people.posture==self.PEOPLE_STANDING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_STANDING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_STANDING].shape[1]] = self.iconMap[self.PEOPLE_STANDING]
                    self.delta_y=self.delta_y+offset_y
                elif people.posture==self.PEOPLE_SITTING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_SITTING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_SITTING].shape[1]] = self.iconMap[self.PEOPLE_SITTING]
                    self.delta_y=self.delta_y+offset_y
                elif people.posture==self.PEOPLE_LYING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_LYING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_LYING].shape[1]] = self.iconMap[self.PEOPLE_LYING]
                    self.delta_y=self.delta_y+offset_y
                elif people.posture==self.UNKNOWN:
                    img[y_offset:y_offset+self.iconMap[self.UNKNOWN].shape[0], x_offset:x_offset+self.iconMap[self.UNKNOWN].shape[1]] = self.iconMap[self.UNKNOWN]
                    self.delta_y=self.delta_y+offset_y

            except Exception as e:
                print 'unable to display icon'+str(e)


    def displayIconHandLeftPosture(self,img,points,people,offset_y,delta_x):
        if len(points)>=2:
            delta=2
            x_offset=(int(round(points[1].x))+delta)+delta_x
            y_offset=self.delta_y
            try:
                if people.handPosture[1]==self.PEOPLE_HAND_CALL:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_LEFT_CALL].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_LEFT_CALL].shape[1]] = self.iconMap[self.PEOPLE_HAND_LEFT_CALL]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[1]==self.PEOPLE_HAND_LEFT_POINTING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_LEFT_POINTING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_LEFT_POINTING].shape[1]] = self.iconMap[self.PEOPLE_HAND_LEFT_POINTING]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[1]==self.PEOPLE_HAND_RIGHT_POINTING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING].shape[1]] = self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[1]==self.PEOPLE_HAND_CROSSED:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_CROSSED].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_CROSSED].shape[1]] = self.iconMap[self.PEOPLE_HAND_CROSSED]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[1]==self.UNKNOWN:
                    img[y_offset:y_offset+self.iconMap[self.UNKNOWN].shape[0], x_offset:x_offset+self.iconMap[self.UNKNOWN].shape[1]] = self.iconMap[self.UNKNOWN]
                    self.delta_y=self.delta_y+offset_y

            except Exception as e:
                print 'unable to display icon'+str(e)


    def displayIconHandRightPosture(self,img,points,people,offset_y,delta_x):
        if len(points)>=2:
            delta=2
            x_offset=(int(round(points[1].x))+delta)+delta_x
            y_offset=self.delta_y
            try:
                if people.handPosture[0]==self.PEOPLE_HAND_CALL:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_CALL].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_CALL].shape[1]] = self.iconMap[self.PEOPLE_HAND_RIGHT_CALL]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[0]==self.PEOPLE_HAND_RIGHT_POINTING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING].shape[1]] = self.iconMap[self.PEOPLE_HAND_RIGHT_POINTING]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[1]==self.PEOPLE_HAND_LEFT_POINTING:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_LEFT_POINTING].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_LEFT_POINTING].shape[1]] = self.iconMap[self.PEOPLE_HAND_LEFT_POINTING]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[0]==self.PEOPLE_HAND_CROSSED:
                    img[y_offset:y_offset+self.iconMap[self.PEOPLE_HAND_CROSSED].shape[0], x_offset:x_offset+self.iconMap[self.PEOPLE_HAND_CROSSED].shape[1]] = self.iconMap[self.PEOPLE_HAND_CROSSED]
                    self.delta_y=self.delta_y+offset_y
                elif people.handPosture[0]==self.UNKNOWN:
                    img[y_offset:y_offset+self.iconMap[self.UNKNOWN].shape[0], x_offset:x_offset+self.iconMap[self.UNKNOWN].shape[1]] = self.iconMap[self.UNKNOWN]
                    self.delta_y=self.delta_y+offset_y

            except Exception as e:
                print 'unable to display icon'+str(e)

    def putTrackedPeopleTxt(self, img, points, tracked_people, (R, G, B)):
        if len(points)>=2:
            y_offset=10
            space_row=2
            cv2.rectangle(img, (int(round(points[0].x)), int(round(points[0].y))-y_offset*4-5), (int(round(points[0].x))+100, int(round(points[0].y))), (0,0,0),  cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, "ID: "+tracked_people.id,(int(round(points[0].x)), int(round(points[0].y))), font, 0.4, (255, 255, 255), 1)
            #delta_y=delta_y+10

            # if(len(label)>10):
            #      label=people.label_id[0:10]
            cv2.putText(img, "WEIGHT: "+str(tracked_people.weight),(int(round(points[0].x)), int(round(points[0].y))-y_offset-space_row), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "TTL: "+str(round(tracked_people.ttl,1)),(int(round(points[0].x)), int(round(points[0].y))-y_offset*2-5), font, 0.4, (255, 255, 255), 1)

            label = tracked_people.label_id
            if len(label) > 10:
                label = label[0:10]

            cv2.putText(img, "LABEL: " + label , (int(round(points[0].x)), int(round(points[0].y)) - y_offset * 3 - 6), font, 0.4, (255, 255, 255), 1)
            delta_y = int(round(points[0].y))
            y_offset=10
            cv2.rectangle(img, (int(round(points[1].x)+2), delta_y), (int(round(points[1].x))+80, delta_y+12*5), (0,0,0),  cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, "SCORE: " + str(round(tracked_people.last_score, 2)),
                        (int(round(points[1].x)), delta_y + 12), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "FACE: "+str(round(tracked_people.last_score_face,2)),(int(round(points[1].x)), delta_y+12*2), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "COl_S: "+str(round(tracked_people.last_score_c_s,2)),(int(round(points[1].x)), delta_y + 12*3), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "COL_T: "+str(round(tracked_people.last_score_c_t,2)),(int(round(points[1].x)), delta_y + 12*4), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "POSE: "+str(round(tracked_people.last_score_pose,2)),(int(round(points[1].x)), delta_y + 12*5), font, 0.4, (255, 255, 255), 1)

    def putTxtIDLabel(self, img,points,people,(R,G,B)):
        if len(points)>=2:
            y_offset=10
            space_row=2

            cv2.rectangle(img, (int(round(points[0].x)), int(round(points[0].y))-y_offset*3-5), (int(round(points[0].x))+100, int(round(points[0].y))), (0,0,0),  cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, "ID: "+people.id,(int(round(points[0].x)), int(round(points[0].y))), font, 0.4, (255, 255, 255), 1)
            #delta_y=delta_y+10


            label=people.label_id

            if(len(label)>10):
                 label=people.label_id[0:10]
            cv2.putText(img, "SCORE: "+str(round(people.label_score*100,1))+"% ",(int(round(points[0].x)), int(round(points[0].y))-y_offset-space_row), font, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "LABEL: "+label,(int(round(points[0].x)), int(round(points[0].y))-y_offset*2-5), font, 0.4, (255, 255, 255), 1)
            #delta_y=delta_y+10
            #cv2.putText(img, "HandPosture:"+str(people.handPosture),(int(round(points[1].x))+delta, delta_y), font, 0.5, (255, 255, 255), 1)
            #delta_y=delta_y+10
            pass


    def putTxtDistance(self, img,points,people,(R,G,B)):
        if len(points)>=2:
            y_offset=10
            cv2.rectangle(img, (int(round(points[1].x)+2), self.delta_y), (int(round(points[1].x))+40, self.delta_y+20), (0,0,0),  cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, str(round(people.distanceEval,2))+"m",(int(round(points[1].x)+2), self.delta_y+12), font, 0.4, (255, 255, 255), 1)

