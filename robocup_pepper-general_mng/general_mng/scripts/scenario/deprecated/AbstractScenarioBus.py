__author__ = 'Jacques Saraydaryan'
from abc import abstractmethod
import rospy
import rostopic
from robocup_msgs.msg import gm_bus_msg
import uuid
from threading import Timer
import time
import threading


class AbstractScenarioBus:
    WAIT_ACTION_STATUS="WAIT_ACTION"
    PENDING_STATUS="PENDING"
    END_FAILURE_STATUS="END_FAILURE"
    END_SUCCESS_STATUS="END_SUCCESS"
    AND_OPERATOR="AND"
    OR_OPERATOR="OR"
    NONE_OPERATOR="NONE"
    _current_operator=NONE_OPERATOR
    _status=PENDING_STATUS
    _gm_sub=None
    _gm_pub=None
    _config=None
    _actionPendingMap={}
    _operatorCheckFunctionMap={}
    #timeout thread
    _t_timer=None
    #check if timeout is trigged (trigged=True)
    _timeout_checker=False

    # how to give a command over the gm bus:
    #
    # msg_xxx_action = gm_bus_msg()
    # msg_xxx_action.action = ""
    # msg_xxx_action.action_id = str(uuid.uuid1())
    # msg_xxx_action.payload = payload_string""
    # msg_xxx_action.result = -1
    # self._gm_bus_pub.publish(msg_xxx_action)
    # rospy.wait_for_message("gm_bus_answer", gm_bus_msg)
    # if self._current_action_result == 4:	# failure
    #     #FIXME need to be completed
    #     print 'xxx failed'
    # elif self._current_action_result == 3: # success
    #     print 'xxx successful'

    def __init__(self,config):
        self._gm_pub = rospy.Publisher("/gm_bus_command", gm_bus_msg, queue_size=1)
        self._gm_sub = rospy.Subscriber("/gm_bus_answer", gm_bus_msg, self.gmBusListener)
        self._config=config
        self._operatorCheckFunctionMap={self.AND_OPERATOR:self.checkAndOperator,self.OR_OPERATOR:self.checkOrOperator,self.NONE_OPERATOR:self.checkNoneOperator}
        self.remapped_topics = dict()
    
    @abstractmethod
    def gmBusListener(self,msg): pass


    def pubNavCmd(self,action,payload):
        msg_gotoA_action = gm_bus_msg()
        msg_gotoA_action.action = action
        msg_gotoA_action.action_id = str(uuid.uuid1())
        msg_gotoA_action.payload = payload
        msg_gotoA_action.result = -1
        # register current action
        self._actionPendingMap[msg_gotoA_action.action_id]=msg_gotoA_action
        # publish current action
        self._gm_pub.publish(msg_gotoA_action)

        rospy.loginfo("### ACTION PENDING : %s",str(msg_gotoA_action).replace('\n',', '))

    
    def pubTtsCmd(self,action,txt):
        self.pubTtsCmdAv(action,txt,"English","NO_WAIT_END")

    def pubTtsCmdAv(self,action,txt,lang,mode):
        msg_tts_action = gm_bus_msg()
        msg_tts_action.action = action
        msg_tts_action.action_id = str(uuid.uuid1())
        msg_tts_action.payload = '{"txt":"'+txt+'","lang":"'+lang+'","mode":"'+mode+'"}'
        msg_tts_action.result = -1
        # register current action
        self._actionPendingMap[msg_tts_action.action_id]=msg_tts_action
        # publish current action
        self._gm_pub.publish(msg_tts_action)

        rospy.loginfo("### ACTION PENDING : %s",str(msg_tts_action).replace('\n',', '))
    
    def waitResult(self,logical_operator):
        self.waitResultTimeout(logical_operator,None)

    def waitResultTimeout(self,logical_operator,maxTimeElapsed):
        #set current operator
        self._current_operator=logical_operator
        if maxTimeElapsed != None:
            #start timer result occurs in self._timeout_checker
            self.startTimeWatchWithTimeOut(maxTimeElapsed)
        #wait current action result --> set by gmBusListener
        while self._status == self.WAIT_ACTION_STATUS and not rospy.is_shutdown() and not self._timeout_checker:
            rospy.sleep(0.1)
        result=False

        if self._timeout_checker:
            rospy.loginfo("# ACTION REACHED TIMEOUT ["+str(maxTimeElapsed)+"s] : %s",str(self._actionPendingMap))
            #reset action pending
            self.resetActionPendingMap()
            self.resetTimer()
            self._status=self.PENDING_STATUS
            return False

        if self._status == self.END_SUCCESS_STATUS:
            result=True
        #reset STATUS
        self.resetActionPendingMap()
        self.resetTimer()
        self._status=self.PENDING_STATUS

        return result

    def checkActionStatus(self,msg):
        if msg.action_id in self._actionPendingMap:
            self._actionPendingMap[msg.action_id]=msg
            result=self._operatorCheckFunctionMap[self._current_operator]()
            info=''
            for key in self._actionPendingMap.keys():
                action=self._actionPendingMap[key]
                #info=info +'|'+'action:'+action.action+' [id:'+action.action_id+',payload:'+action.payload+'] :'+str(action.result)+'|'
                info=info + 'action:'+action.action+' [id:'+action.action_id+'],'
            if result == 3:
                rospy.loginfo("# ACTION SUCCEED : %s",str(info))
                #rest current action list
                self.resetActionPendingMap()
                self._status = self.END_SUCCESS_STATUS
            elif result == 4:
                rospy.logwarn("# ACTION FAILED : %s",str(info))
                #rest current action list
                self.resetActionPendingMap()
                self._status = self.END_FAILURE_STATUS
            elif result==-1:
                rospy.loginfo("# Waiting other Action: %s",str(info))

            
    def checkAndOperator(self):
        waitingAction=False
        for key in self._actionPendingMap.keys():
            if self._actionPendingMap[key].result != 3:
                return 4
            if self._actionPendingMap[key].result == -1:
                waitingAction=True
        if waitingAction:
            return -1
        else:
            return 3

    def checkOrOperator(self):
        waitingAction=False
        for key in self._actionPendingMap.keys():
            if self._actionPendingMap[key].result == 3:
                return True
            if self._actionPendingMap[key].result == -1:
                waitingAction=True
        if waitingAction:
            return -1
        else:
            return 4

    def checkNoneOperator(self):
        return self.checkOrOperator()


    def startTimeWatchWithTimeOut(self,maxTimeElapsed):
        self._t_timer = Timer(maxTimeElapsed, self.timoutCallBack)
        self._t_timer.start()

    def timoutCallBack(self):
        self._timeout_checker=True

    def resetTimer(self):
        try:
            self._t_timer.cancel()
        except Exception as e:
            rospy.loginfo("Unable to reset timer: %s" % e)
        self._timeout_checker=False

    def resetActionPendingMap(self):
         self._actionPendingMap={}

    def remap_topic(self, source_topic, target_topic, only_one_msg=False, freq=-1.0, queue_size=10):
        if (source_topic, target_topic) in self.remapped_topics:
            remap_data = self.remapped_topics[(source_topic, target_topic)]
            remap_data["only_one_msg"] = only_one_msg
            remap_data["freq"] = freq
            remap_data["duration"] = 1.0 / freq
            remap_data["last_call_time"] = time.time()
        else:
            topic_msg_class = rostopic.get_topic_class(source_topic)[0]
            publisher = rospy.Publisher(target_topic, topic_msg_class, queue_size=queue_size)
            lock = threading.Lock()
            lock.acquire()
            self.remapped_topics[(source_topic, target_topic)] = {"only_one_msg": only_one_msg,
                                                                  "freq": freq,
                                                                  "duration": 1.0 / freq,
                                                                  "publisher": publisher,
                                                                  "last_call_time": time.time()}
            lock.release()
            subscriber = rospy.Subscriber(
                source_topic, topic_msg_class, callback=self.remap_callback, callback_args=(source_topic, target_topic))
            lock.acquire()
            self.remapped_topics[(source_topic, target_topic)]["subscriber"] = subscriber
            lock.release()

    def remap_callback(self, msg, (source_topic, target_topic)):
        this_call_time = time.time()
        lock = threading.Lock()
        lock.acquire()
        remap_data = self.remapped_topics[(source_topic, target_topic)]
        lock.release()
        if remap_data["freq"] < 0.0:
            remap_data["publisher"].publish(msg)
            if remap_data["only_one_msg"]:
                self.unremap_topic(source_topic, target_topic)
        elif this_call_time - remap_data["last_call_time"] >= remap_data["duration"]:
            remap_data["publisher"].publish(msg)
            if remap_data["only_one_msg"]:
                self.unremap_topic(source_topic, target_topic)
                remap_data["last_call_time"] = this_call_time

    def unremap_topic(self, source_topic, target_topic):
        lock = threading.Lock()
        lock.acquire()
        remap_data = self.remapped_topics[(source_topic, target_topic)]
        lock.release()
        remap_data["subscriber"].unregister()
        lock = threading.Lock()
        lock.acquire()
        del self.remapped_topics[(source_topic, target_topic)]
        lock.release()
