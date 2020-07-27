__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import uuid
import time
import random
import actionlib

from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenario import AbstractScenario


from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from dialogue_hri_srvs.srv import MoveSound, MoveTurn, TakePicture





class SPRV2Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None
    _peopleMetaMap=''
    HEAD_PITCH_FOR_PEOPLE_DETECTION=0.0
    HEAD_YAW_FOR_PEOPLE_DETECTION=0.0
    DEFAULT_QUESTION_LOCATION=[]
    QUESTION_GENDER_MALE='gender_male'
    QUESTION_GENDER_FEMALE='gender_female'
    QUESTION_POSE_STANDING='pose_standing'
    QUESTION_POSE_SITTING='pose_sitting'
    QUESTION_POSE_WAVING='pose_waving'

    QUESTION_POSE_RAISING_LEFT='pose_raising_left'
    QUESTION_POSE_RAISING_RIGHT='pose_raising_right'
    QUESTION_POSE_POINTING_RIGHT='pose_pointing_right'
    QUESTION_POSE_POINTING_LEFT='pose_pointing_left'
    QUESTION_POSE_CROSSING='pose_crossing'

    QUESTION_POSE_LYING='pose_lying'
    QUESTION_PEOPLE='people'
    QUESTION_COLOR_RED='color_red'
    QUESTION_COLOR_BLUE='color_blue'
    QUESTION_COLOR_WHITE='color_white'
    QUESTION_COLOR_BLACK='color_black'
    QUESTION_COLOR_GREEN='color_green'
    QUESTION_COLOR_YELLOW='color_yellow'   
    
    #DEFAULT_OBJ_LABEL=[]
    #DEFAULT_OBJECT_MEMORY_LOCATION="Robocup/objects"

    def __init__(self,config):
        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)
        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)

        try:
            self.question_memory_location=config['question_memory_location']
        except Exception as e:
            rospy.logwarn("no config value for question_memory_location use default:"+str(self.DEFAULT_QUESTION_LOCATION))
            self.question_memory_location=self.DEFAULT_QUESTION_LOCATION
#
        #try:
        #    self.object_memory_location=config['object_memory_location']
        #except Exception as e:
        #    rospy.logwarn("no config value for object_memory_location use default:"+str(self.DEFAULT_OBJECT_MEMORY_LOCATION))
        #    self.object_memory_location=self.DEFAULT_OBJECT_MEMORY_LOCATION

            
        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)

        try:
            rospy.wait_for_service('move_turn_service',5)
            rospy.loginfo("end service move_turn_service wait time")
            self._moveTurn = rospy.ServiceProxy('move_turn_service', MoveTurn)
        except Exception as e:
            rospy.logerr("Service move_turn_service call failed: %s" % e)

        try:
            rospy.wait_for_service('activate_move_sound_service',5)
            rospy.loginfo("end service activate_move_sound_service wait time")
            self._activateMoveSound = rospy.ServiceProxy('activate_move_sound_service', MoveSound)
        except Exception as e:
            rospy.logerr("Service activate_move_sound_service call failed: %s" % e)


        try:
            rospy.wait_for_service('take_picture_service',5)
            rospy.loginfo("end service take_picture_service wait time")
            self._activateMoveSound = rospy.ServiceProxy('take_picture_service', TakePicture)
        except Exception as e:
            rospy.logerr("Service take_picture_service call failed: %s" % e)


            #return
        self._peopleMetaMap={}
        try:
            self.question_memory_location[self.QUESTION_PEOPLE]
            self.question_memory_location[self.QUESTION_GENDER_MALE]
            self.question_memory_location[self.QUESTION_GENDER_FEMALE]
            self.question_memory_location[self.QUESTION_POSE_STANDING]
            self.question_memory_location[self.QUESTION_POSE_SITTING]
            self.question_memory_location[self.QUESTION_POSE_LYING]
            self.question_memory_location[self.QUESTION_POSE_WAVING]

            self.question_memory_location[self.QUESTION_POSE_RAISING_LEFT]
            self.question_memory_location[self.QUESTION_POSE_RAISING_RIGHT]
            self.question_memory_location[self.QUESTION_POSE_POINTING_RIGHT]
            self.question_memory_location[self.QUESTION_POSE_POINTING_LEFT]
            self.question_memory_location[self.QUESTION_POSE_CROSSING]

            self.question_memory_location[self.QUESTION_COLOR_BLACK]
            self.question_memory_location[self.QUESTION_COLOR_BLUE]
            self.question_memory_location[self.QUESTION_COLOR_GREEN]
            self.question_memory_location[self.QUESTION_COLOR_RED]
            self.question_memory_location[self.QUESTION_COLOR_WHITE]
            self.question_memory_location[self.QUESTION_COLOR_YELLOW]

            self.resetPeopleMetaMap()
        except Exception as e:
            rospy.logerr("BAD CONFIG FILE MISSING MEMORY LOCATION KEY:"+str(e))



    def startScenario(self):
        self.resetPeopleMetaMap()
        try:
            rospy.loginfo("")
            rospy.loginfo("######################################")
            rospy.loginfo("Starting the SPRV1 Scenario...")
            rospy.loginfo("######################################")
        
            #TOO make the logic of the scenario

            self.sendTtsOrderAction("TTS","I will the SPR start in...","NO_WAIT_END","English",60.0)
            index=10
            for i in range(0,11):
                # tell current time 
                self.sendTtsOrderAction("TTS",str(index),"NO_WAIT_END","English",60.0)
                index=index-1

                #Sleep before turning
                rospy.sleep(0.6)

            #turn of 180 degre before detecting people
            self.moveTurn(3.14)

            # set the head position for detecting people
            self.moveheadPose(self.HEAD_PITCH_FOR_PEOPLE_DETECTION,self.HEAD_YAW_FOR_PEOPLE_DETECTION,True)
            rospy.sleep(2.0)

            #take a picture
            self.takePicture('/tmp/imageFrontPepper.png')
            
            #Start detecting People
            orderState0,result0=self.detectMetaPeopleFromImgPath('/tmp/imageFrontPepper.png',30)
            rospy.loginfo(result0)

            nbOfPeople=0
            if orderState0 == 3:
                try:
                    #process people attribute to save elts to ALMEMORY
                    self.processResult(result0)
                    nbOfPeople=len(result0.peopleMetaList.peopleList)
                except Exception as e:
                    rospy.logwarn(e)

            self.moveheadPose(self.HEAD_PITCH_FOR_PEOPLE_DETECTION,self.HEAD_YAW_FOR_PEOPLE_DETECTION,False)
            rospy.sleep(2.0)

            # say the number of people
            self.sendTtsOrderAction("TTS","In the crowd, i detect "+str(nbOfPeople)+" people","NO_WAIT_END","English",60.0)
            self.sendTtsOrderAction("TTS","In the crowd, "+str(nbOfPeople-1)+" people are men","NO_WAIT_END","English",60.0)

            #Tell pepper process is finished
            orderState5,result5=self.sendDialogueOrderAction("SPR/ProcessPeopleFinished","",5.0)

            self.sendTtsOrderAction("TTS","I use my super power to compute all recognition combinaisons, please wait ...","NO_WAIT_END","English",60.0)
            # wait
            rospy.sleep(10.0)
            self.sendTtsOrderAction("TTS","I am ready to answer to your questions","NO_WAIT_END","English",60.0)

            #Activate sound location
            self.activateSoundLocation(True)
        except Exception as e:
            rospy.logwarn(str(e))




    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):
        
        self._enableNavAction=False
        self._enableTtsAction=True
        self._enableDialogueAction=True
        self._enableAddInMemoryAction=True
        self._enableObjectDetectionMngAction=True
        self._enableMultiplePeopleDetectionAction=True
        
        AbstractScenarioAction.configure_intern(self)

    def moveheadPose(self,pitch_value,yaw_value,track):
        try:
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            result=self._moveHeadPose(pitch_value,yaw_value,track)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return

    def moveTurn(self,rad):
        try:
            self._moveTurn = rospy.ServiceProxy('move_turn_service', MoveTurn)
            result=self._moveTurn(rad)
        except Exception as e:
            rospy.logerr("Service move_turn_service call failed: %s" % e)
            return

    def activateSoundLocation(self,isactivated):
        try:
            self._activateMoveSound = rospy.ServiceProxy('activate_move_sound_service', MoveSound)
            result=self._activateMoveSound(isactivated)
        except Exception as e:
            rospy.logerr("Service activate_move_sound_service call failed: %s" % e)
            return
    
    def takePicture(self,image_path):
        try:
            self._takePictureSrv = rospy.ServiceProxy('take_picture_service', TakePicture)
            result=self._takePictureSrv(image_path)
            return
        except Exception as e:
            rospy.logerr("Service take_picture_service call failed: %s" % e)
            return

    def processResult(self,result):
        if len(result.peopleMetaList.peopleList) == 0:
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_PEOPLE],str(0),5.0)
        else:
            self._peopleMetaMap[self.QUESTION_PEOPLE]=len(result.peopleMetaList.peopleList)
            for person in result.peopleMetaList.peopleList:
               
                self._peopleMetaMap[self.QUESTION_GENDER_MALE]=0
                self._peopleMetaMap[self.QUESTION_GENDER_FEMALE]=0
                if person.posture=='Standing' or person.posture=='Undefined':
                    self._peopleMetaMap[self.QUESTION_POSE_STANDING]+=1
                    
                elif person.posture=='Sitting':
                    self._peopleMetaMap[self.QUESTION_POSE_SITTING]+=1
                    
                elif person.posture=='Lying':
                    self._peopleMetaMap[self.QUESTION_POSE_LYING]+=1
                    
                if len(person.handPosture)==2:
                    for i in range(0,2):
                        if person.handPosture[i] == 'Call' :
                            self._peopleMetaMap[self.QUESTION_POSE_WAVING]+=1
                            if i == 0:
                                self._peopleMetaMap[self.QUESTION_POSE_RAISING_LEFT]+=1
                            else:
                                self._peopleMetaMap[self.QUESTION_POSE_RAISING_RIGHT]+=1
                        elif person.handPosture[i].find('Pointing')>=0:
                            if i == 0:
                                self._peopleMetaMap[self.QUESTION_POSE_POINTING_RIGHT]+=1
                            else:
                                self._peopleMetaMap[self.QUESTION_POSE_POINTING_LEFT]+=1

                    if person.handPosture[0] == 'Crossed' :
                        self._peopleMetaMap[self.QUESTION_POSE_CROSSING]+=1
                
                if person.shirt_color_name=='BLACK':
                    self._peopleMetaMap[self.QUESTION_COLOR_BLACK]+=1
                elif 'BLUE' in person.shirt_color_name or 'CYAN' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_BLUE]+=1
                elif 'GREEN' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_GREEN]+=1
                elif 'RED' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_RED]+=1
                elif 'WHITE' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_WHITE]+=1
                elif 'YELLOW' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]+=1

            self.addInPepperMemory(self.question_memory_location[self.QUESTION_PEOPLE],str(self._peopleMetaMap[self.QUESTION_PEOPLE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_STANDING],str(self._peopleMetaMap[self.QUESTION_POSE_STANDING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_SITTING],str(self._peopleMetaMap[self.QUESTION_POSE_SITTING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_LYING],str(self._peopleMetaMap[self.QUESTION_POSE_LYING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_WAVING],str(self._peopleMetaMap[self.QUESTION_POSE_WAVING]),5.0)

            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_RAISING_LEFT],str(self._peopleMetaMap[self.QUESTION_POSE_RAISING_LEFT]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_RAISING_RIGHT],str(self._peopleMetaMap[self.QUESTION_POSE_RAISING_RIGHT]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_POINTING_RIGHT],str(self._peopleMetaMap[self.QUESTION_POSE_POINTING_RIGHT]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_POINTING_LEFT],str(self._peopleMetaMap[self.QUESTION_POSE_POINTING_LEFT]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_CROSSING],str(self._peopleMetaMap[self.QUESTION_POSE_CROSSING]),5.0)

            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_BLACK],str(self._peopleMetaMap[self.QUESTION_COLOR_BLACK]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_BLUE],str(self._peopleMetaMap[self.QUESTION_COLOR_BLUE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_GREEN],str(self._peopleMetaMap[self.QUESTION_COLOR_GREEN]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_RED],str(self._peopleMetaMap[self.QUESTION_COLOR_RED]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_WHITE],str(self._peopleMetaMap[self.QUESTION_COLOR_WHITE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_YELLOW],str(self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]),5.0)

    def resetPeopleMetaMap(self):
        try:
           self._peopleMetaMap[self.QUESTION_PEOPLE]=0
           self._peopleMetaMap[self.QUESTION_GENDER_MALE]=0
           self._peopleMetaMap[self.QUESTION_GENDER_FEMALE]=0
           self._peopleMetaMap[self.QUESTION_POSE_STANDING]=0
           self._peopleMetaMap[self.QUESTION_POSE_SITTING]=0
           self._peopleMetaMap[self.QUESTION_POSE_LYING]=0
           self._peopleMetaMap[self.QUESTION_POSE_WAVING]=0

           self._peopleMetaMap[self.QUESTION_POSE_RAISING_LEFT]=0
           self._peopleMetaMap[self.QUESTION_POSE_RAISING_RIGHT]=0
           self._peopleMetaMap[self.QUESTION_POSE_POINTING_RIGHT]=0
           self._peopleMetaMap[self.QUESTION_POSE_POINTING_LEFT]=0
           self._peopleMetaMap[self.QUESTION_POSE_CROSSING]=0

           self._peopleMetaMap[self.QUESTION_COLOR_BLACK]=0
           self._peopleMetaMap[self.QUESTION_COLOR_BLUE]=0
           self._peopleMetaMap[self.QUESTION_COLOR_GREEN]=0
           self._peopleMetaMap[self.QUESTION_COLOR_RED]=0
           self._peopleMetaMap[self.QUESTION_COLOR_WHITE]=0
           self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]=0
        except Exception as e:
           rospy.logwarn("Error resetting _peopleMetaMap"+str(e))