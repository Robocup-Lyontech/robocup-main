__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import uuid
import time
import random
import actionlib
import yaml

from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenario import AbstractScenario


from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from pepper_pose_for_nav.srv import MoveHeadAtPosition





class GPRSV1Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None
    HEAD_PITCH_FOR_SPEECH_POSE=0.20
    HEAD_PITCH_FOR_NAV_POSE= 0.5
    HEAD_YAW_CENTER=0.0
    DEFAULT_OBJ_LABEL=[]
    DEFAULT_OBJECT_MEMORY_LOCATION="Cocktail/Objects"
    DEFAULT_GOTO_EVERYWHERE="everywhere"
    DEFAULT_TARGET_OBJECT="object"
    DEFAULT_TARGET_PEOPLE="people"

    def __init__(self,config):
        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)


        try:
            self.obj_labels=config['labels']
        except Exception as e:
            rospy.logwarn("no config value for obj_label use default:"+str(self.DEFAULT_OBJ_LABEL))
            self.obj_labels=self.DEFAULT_OBJ_LABEL

        try:
            self.object_memory_location=config['object_memory_location']
        except Exception as e:
            rospy.logwarn("no config value for object_memory_location use default:"+str(self.DEFAULT_OBJECT_MEMORY_LOCATION))
            self.object_memory_location=self.DEFAULT_OBJECT_MEMORY_LOCATION




        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the TEST_COCKTAIL_PARTY_V1 Scenario...")
        rospy.loginfo("######################################")

        self._location_beacon={}
        self._location_room={}
        self._location_rooms_list=[]
        self._location_rooms_togo=[]
        self._location_beacon_by_rooms_map={}
        self._location_rooms_list=[]

        self._location_rooms_list.append(self.DEFAULT_GOTO_EVERYWHERE)
        self._location_rooms_list.append('room_kitchen')
        self._location_rooms_list.append('room_living_room')
        self._location_rooms_list.append('room_dining_room')
        self._location_rooms_list.append('room_bedroom')
        self._location_rooms_list.append('room_cooridoor')
        self._location_rooms_list.append('room_entrance')
        self._location_rooms_list.append('room_exit')

        #self._location_rooms_togo.append(self.DEFAULT_GOTO_EVERYWHERE)
        self._location_rooms_togo.append('GPRS_PEOPLE_KITCHEN_It0')
        self._location_rooms_togo.append('GPRS_PEOPLE_LIVINGROOM_It0')
        self._location_rooms_togo.append('GPRS_PEOPLE_DININGROOM_It0')
        self._location_rooms_togo.append('GPRS_PEOPLE_BEDROOM_It0')
        self._location_rooms_togo.append('GPRS_PEOPLE_ENTRANCE_It0')

        #self._location_room['everywhere']='everywhere'
        self._location_room['room_kitchen']='GPRS_PEOPLE_KITCHEN_It0'
        self._location_room['room_living_room']='GPRS_PEOPLE_LIVINGROOM_It0'
        self._location_room['room_dining_room']='GPRS_PEOPLE_DININGROOM_It0'
        self._location_room['room_bedroom']='GPRS_PEOPLE_BEDROOM_It0'
        self._location_room['room_cooridoor']='GPRS_PEOPLE_ENTRANCE_It0'
        self._location_room['room_entrance']='GPRS_START_It0'
        self._location_room['room_exit']='GPRS_PEOPLE_EXIT_It1'
#
        self._location_beacon['beacon_storage_table']='GPRS_OBJECT_KITCHEN_It1'
        self._location_beacon['beacon_bookcase']='GPRS_OBJECT_LIVINGROOM_It1'
        self._location_beacon['beacon_cupboard']='GPRS_OBJECT_KITCHEN_It0'
        self._location_beacon['beacon_desk']='GPRS_OBJECT_BEDROOM_It1'
        self._location_beacon['beacon_dinning_table']='GPRS_OBJECT_DININGROOM_It0'
        self._location_beacon['beacon_end_table']='GPRS_OBJECT_LIVINGROOM_It0'
        self._location_beacon['beacon_side_table']='GPRS_OBJECT_BEDROOM_It0'
        self._location_beacon['beacon_counter']='GPRS_OBJECT_KITCHEN_It2'
        self._location_beacon['beacon_sink']='GPRS_OBJECT_KITCHEN_It3'
        self._location_beacon['beacon_couch']='GPRS_OBJECT_LIVINGROOM_It0'
        self._location_beacon['beacon_entrance']='GPRS_PEOPLE_ENTRANCE_It0'
        self._location_beacon['beacon_dishwasher']='GPRS_OBJECT_KITCHEN_It2'

        self._location_beacon_by_rooms_map['room_kitchen']=[]
        self._location_beacon_by_rooms_map['room_living_room']=[]
        self._location_beacon_by_rooms_map['room_dining_room']=[]
        self._location_beacon_by_rooms_map['room_bedroom']=[]
        self._location_beacon_by_rooms_map['room_cooridoor']=[]
        self._location_beacon_by_rooms_map['room_entrance']=[]

        self._location_beacon_by_rooms_map['room_kitchen'].append('GPRS_OBJECT_KITCHEN_It0')
        self._location_beacon_by_rooms_map['room_kitchen'].append('GPRS_OBJECT_KITCHEN_It1')
        self._location_beacon_by_rooms_map['room_kitchen'].append('GPRS_OBJECT_KITCHEN_It2')
        self._location_beacon_by_rooms_map['room_kitchen'].append('GPRS_OBJECT_KITCHEN_It3')
        self._location_beacon_by_rooms_map['room_living_room'].append('GPRS_OBJECT_LIVINGROOM_It0')
        self._location_beacon_by_rooms_map['room_living_room'].append('GPRS_OBJECT_LIVINGROOM_It1')
        self._location_beacon_by_rooms_map['room_dining_room'].append('GPRS_OBJECT_DININGROOM_It0')
        self._location_beacon_by_rooms_map['room_bedroom'].append('GPRS_OBJECT_BEDROOM_It0')
        self._location_beacon_by_rooms_map['room_bedroom'].append('GPRS_OBJECT_BEDROOM_It1')
        self._location_beacon_by_rooms_map['room_cooridoor'].append('GPRS_PEOPLE_ENTRANCE_It0')
        self._location_beacon_by_rooms_map['room_entrance'].append('GPRS_PEOPLE_ENTRANCE_It0')


        ##TOO make the logic of the scenario
#
        ##### 3-NAVIGATE TO OPERATOR AREA
        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It0",60.0)
        #orderState01=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It1",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_BEDROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_BEDROOM_It1",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_DININGROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_KITCHEN_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_KITCHEN_It1",60.0)
        #orderState02=self.sendNavOrderActself.moveheadFromItInfo(current_location)ion("NP","CRRCloseToGoal","GPRS_OBJECT_KITCHEN_It2",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_KITCHEN_It3",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_LIVINGROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_LIVINGROOM_It1",60.0)
        #
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_BEDROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_DININGROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_ENTRANCE_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_LIVINGROOM_It0",60.0)


        ## TO REMOVE !!!!!!!! FOR TEST ONLY
        #payload='{"gotoroom":"room_kitchen", "target_type":"people", "target":"carrot"}'
        #jsonContent = yaml.safe_load(payload)
        #gotToOrder=jsonContent['gotoroom'self.moveheadFromItInfo(current_location)]
        #current_target=jsonContent['target']
        #current_type=jsonContent['target_type']
        ## END TO REMOVE

        #self.moveheadFromItInfo('GPRS_START_It0')

        #orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It0",60.0)
        #orderState01=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It1",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It3",60.0)


        #### XX-DETECTION OBJECTS
        orderState7,result7=self.getObjectInFrontRobot([], True, 60.0)
        self.sendTtsOrderAction("TTS","I found the"+str(result7.labelList)+" objects !!! Go back to the operator" ,"NO_WAIT_END","English",60.0)

        return

        self.sendTtsOrderAction("TTS","Ready to get your order!" ,"NO_WAIT_END","English",60.0)
                ### 5-INFORM NAOQI TO START TO GET COMMAND
        orderState1,result1=self.sendDialogueOrderAction("GPRS/OrdersStart","GPRS/OrdersFinish",20.0*1) #CAUTIOn UPDATE THE WAIT DURATION
        rospy.loginfo(result1)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_OBJECT_LIVINGROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_BEDROOM_It0",60.0)
        #orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_PEOPLE_ENTRANCE_It0",60.0)

        try:
                jsonContent = yaml.safe_load(result1.payload)
                gotToOrder=jsonContent['gotoroom']
                current_target=jsonContent['target']
                current_type=jsonContent['target_type']
        except Exception as e:
                rospy.logwarn("UNABLE TO PARSE ORDER e:"+str(e))
                self.sendTtsOrderAction("TTS","Ok , I will find your favorite object... " ,"NO_WAIT_END","English",60.0)
                #payload='{"gotoroom":"everywhere", "target_type":"people", "target":"coke"}'
                payload='{"gotoroom":"room_bedroom", "target_type":"object", "target":"chocolate_drink"}'
                jsonContent = yaml.safe_load(payload)
                gotToOrder=jsonContent['gotoroom']
                current_target=jsonContent['target']
                current_type=jsonContent['target_type']

        if self.DEFAULT_GOTO_EVERYWHERE == gotToOrder:
                current_target=jsonContent['target']
                current_type=jsonContent['target_type']
                self.processEveryWhere(current_type,current_target)

        else:
                if current_type ==self.DEFAULT_TARGET_OBJECT:
                        try:
                                current_destination=self._location_beacon_by_rooms_map[gotToOrder][1]
                        except Exception as e:
                                rospy.loginfo("WARNING NO DESTINATION TO REACH e:"+str(e))

                else:
                        isBeacon=False
                        try:
                                current_destination=self._location_room[gotToOrder]
                        except Exception as e:
                                rospy.loginfo("Destination is not a room try a beacon destination :"+str(gotToOrder))
                                current_destination=self._location_beacon[gotToOrder]
                                isBeacon=True
                try:
                        orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal",current_destination,60.0)
                        self.moveheadFromItInfo(current_destination)
                except Exception as e:
                        rospy.loginfo("WARNING NO DESTINATION TO REACH")

                current_type=jsonContent['target_type']
                if self.DEFAULT_TARGET_OBJECT == current_type:
                        current_target=jsonContent['target']
                        obj_labels=[]
                        #obj_labels.append(current_target)
                        object_found=False
                        for current_location in  self._location_beacon_by_rooms_map[gotToOrder]:
                                #### XX-DETECTION OBJECTS
                                orderState7,result7=self.getObjectInFrontRobot(obj_labels, True, 60.0)
                                rospy.loginfo("#### OBJECT DETECTED ####")
                                rospy.loginfo(result7)
                                try:
                                        if len(result7.labelList) > 0 :
                                                #result0.labelList
                                                ### Say the object was found

                                                self.sendTtsOrderAction("TTS","I found the"+str(result7.labelList)+" objects !!! Go back to the operator" ,"NO_WAIT_END","English",60.0)
                                                #self.sendTtsOrderAction("TTS","I found the"+str(current_target)+" object !!! Go back to the operator" ,"NO_WAIT_END","English",60.0)
                                                #### XX-INFORM NAOQI OBJECT FOUND
                                                orderState1,result1=self.sendDialogueOrderAction("GPRS/FindObjectStart","",10.0*1)
                                                #### XX-GO BACK TO OPERATOR
                                                orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
                                                object_found=True
                                                break
                                except Exception as e:
                                         rospy.logwarn("UNABLE TO GET INFO FROm OBJECT e:"+str(e))

                        if not object_found:
                                self.sendTtsOrderAction("TTS","I so sorry, I did not found the"+str(current_target)+" object, Go back to the operator" ,"NO_WAIT_END","English",60.0)
                                orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)

                if self.DEFAULT_TARGET_PEOPLE == current_type:
                        #Start detecting People
                        ### SET HEAD FOR RECOGNITION
                        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,self.HEAD_YAW_CENTER,True)

                        orderState0,result0=self.detectMetaPeopleFromImgTopic(30)
                        rospy.loginfo(result0)

                        try:
                                #if len(result.peopleMetaList.peopleList) >0:
                                if self.checkPeopleWithLeg(result0.peopleMetaList.peopleList) > 0:
                                        self.sendTtsOrderAction("TTS","I found someone !!" ,"NO_WAIT_END","English",60.0)
                                        #### XX-INFORM NAOQI OBJECT FOUND
                                        orderState1,result1=self.sendDialogueOrderAction("GPRS/FindPeopleStart","GPRS/FindPeopleFinish",60.0*3)

                                        #FIXME TO BE COMPLETED
                                        # if result1.payload ==1 :

                                        #### XX-GO BACK TO OPERATOR
                                        orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
                        except Exception as e:
                                rospy.logwarn("UNABLE TO GET INFO FROm PEOPLE e:"+str(e))


    def processEveryWhere(self,target_type,target):
        if self.DEFAULT_TARGET_PEOPLE == target_type:
                    for current_location in self._location_rooms_togo:
                        #### XX-GO TO LOCATION
                        orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal",current_location,60.0)

                        ### SET HEAD FOR RECOGNITION
                        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE,self.HEAD_YAW_CENTER,True)

                        #Start detecting People

                        orderState0,result0=self.detectMetaPeopleFromImgTopic(30)
                        try:
                                #if len(result.peopleMetaList.peopleList) >0:
                                if self.checkPeopleWithLeg(result0.peopleMetaList.peopleList) > 0:
                                        self.sendTtsOrderAction("TTS","I found someone !!" ,"NO_WAIT_END","English",60.0)
                                        #### XX-INFORM NAOQI OBJECT FOUND
                                        orderState1,result1=self.sendDialogueOrderAction("GPRS/FindPeopleStart","GPRS/FindPeopleFinish",60.0*3)

                                        try:
                                                ## Check result1
                                               if result1.payload == 'SUCCESS':
                                                        ###
                                                        self.sendTtsOrderAction("TTS","I do my job !, I go back to the operator" ,"NO_WAIT_END","English",60.0)
                                                        #### XX-GO BACK TO OPERATOR
                                                        orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
                                                        break
                                        except Exception as e:
                                                rospy.logwarn(" Unable to process received data e:"+str(e))


                        except Exception as e:
                                rospy.logwarn("UNABLE TO PROCESS PEOPLE e:"+str(e))
        if self.DEFAULT_TARGET_OBJECT == target_type:
                object_found=False
                obj_labels=[]
                obj_labels.append(target)
                for current_location in self._location_rooms_togo:
                        #### XX-GO TO LOCATION
                        orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal",current_location,60.0)
                        self.moveheadFromItInfo(current_location)
                        #### XX-DETECTION OBJECTS
                        orderState7,result7=self.getObjectInFrontRobot(obj_labels, True, 60.0)
                        rospy.loginfo("#### OBJECT DETECTED ####")
                        rospy.loginfo(result7)
                        try:
                                if len(result7.labelList) > 0 :
                                        #result0.labelList
                                        ### Say the object was found
                                        self.sendTtsOrderAction("TTS","I found the"+str(target)+" object !!! Go back to the operator" ,"NO_WAIT_END","English",60.0)
                                        #### XX-INFORM NAOQI OBJECT FOUND
                                        orderState1,result1=self.sendDialogueOrderAction("GPRS/FindObjectStart","",60.0*3)
                                        #### XX-GO BACK TO OPERATOR
                                        orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)
                                        object_found=True
                                        break
                        except Exception as e:
                                rospy.logwarn("UNABLE TO PROCESS Object")
                if not object_found:
                        self.sendTtsOrderAction("TTS","I so sorry, I did not found the"+str(target)+" object, Go back to the operator" ,"NO_WAIT_END","English",60.0)


                orderState02=self.sendNavOrderAction("NP","CRRCloseToGoal","GPRS_START_It2",60.0)


    def checkPeopleWithLeg(self,peoplelist):
        nbPeople=0
        if len(peoplelist) == 0:
            return 0
        else:
            for person in peoplelist:
                if person.trouser_color_name != 'NONE':
                        nbPeople=nbPeople+1
        return nbPeople


    def gmBusListener(self,msg):
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):

        self._enableNavAction=True
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

    def moveheadFromItInfo(self,it_name):
            try:
                    self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)
                    current_it=self._getPoint_service(it_name)
                    #rospy.loginfo(current_it)
                    self.moveheadPose(current_it.head_pitch,current_it.head_yaw,true)
            except Exception as e:
                    rospy.logwarn("Unable to move head to data from It point info e:"+str(e))
