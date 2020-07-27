import rospy


class PersonMetaInfo():
    SHIRT_RECT="SHIRT_RECT"
    TROUSER_RECT="TROUSER_RECT"
    PERSON_RECT="PERSON_RECT"

    id=-1
    posture=''
    handCall=False
    boundingBoxMap={}
    colorNameMap={}
    colorRGBMap={}
    distanceEval=0
    label_id=''
    label_score=0.0
    pose=""

    def __init__(self,id):
        self.id=id
        self.boundingBoxMap={}
        self.colorNameMap={}
        self.colorRGBMap={}
        self.colorList={}

    def setBoundingBox(self,nameKey,ptList):
        try:
            self.boundingBoxMap[nameKey]=ptList
        except Exception as e:
            rospy.logwarn("Unable to add bounding box:"+str(e))

    def setPosition(self,pose):
        self.pose = pose

    def getBoundingBox(self,nameKey):
        value=[]
        try:
            value=self.boundingBoxMap[nameKey]
        except Exception as e:
            rospy.logwarn("Unable to get bounding box:"+str(e))
        return value

    def setMainColor(self,nameKey,color_label,rgb_value):
        try:
            self.colorNameMap[nameKey]=color_label
            self.colorRGBMap[nameKey]=rgb_value
        except Exception as e:
            rospy.logwarn("Unable to add color:"+str(e))
    def getMainColor(self,nameKey):
        value='NONE'
        try:
            value=self.colorNameMap[nameKey]
            
        except Exception as e:
            rospy.logwarn("Unable to get color:"+str(e))
        return value

    def setColorList(self,nameKey, colorDList):
        try:
            self.colorList[nameKey]=colorDList
        except Exception as e:
            rospy.logwarn("Unable to add color list:"+str(e))

    def getColorList(self,nameKey):
        value=[]
        try:
            value=self.colorList[nameKey]
            
        except Exception as e:
            rospy.logwarn("Unable to get color:"+str(e))
        return value

    def __str__(self):
        return '{"boundingBoxMap":'+str(self.boundingBoxMap)+'",colorNameMap":'+str(self.colorNameMap)+'",colorRGBMap":'+str(self.colorRGBMap)+'",distanceEval":'+str(self.distanceEval)+'",label_id":'+str(self.label_id)+'",posture":'+str(self.posture)+'",handCall":'+str(self.handCall)+'",id":'+str(self.id)+'}'

