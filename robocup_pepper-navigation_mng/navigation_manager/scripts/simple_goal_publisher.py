#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class GoToDefPose():
    pub = ''

    def __init__(self):
        rospy.init_node('GoToArenaPose', anonymous=True)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.ChoosePose()
    
    def ChoosePose(self):
        while not rospy.is_shutdown():
            Pose = PoseStamped()
            print
            print('Options : ')
            print(' - 0 : Quit')
            print(' - 1 : Entrance')
            print(' - 2 : Kitchen')
            print(' - 3 : Charging Place')
            answer = input('Where do you want to send the robot ? ')
            if(answer == 0):
                break
            elif(answer == 1): #Entrance
                Pose.header.frame_id = 'map'
                Pose.pose.position.x = -3.09927248955
                Pose.pose.position.y = 0.147106766701
                Pose.pose.orientation.z = 0.965981851705
                Pose.pose.orientation.w = 0.25860986481
            elif (answer == 2): #kitchen
                Pose.header.frame_id = 'map'
                Pose.pose.position.x = -3.16048765182
                Pose.pose.position.y = 2.15104579926
                Pose.pose.orientation.z = 0.978578773184
                Pose.pose.orientation.w = 0.205872739024
            elif(answer == 3): #charging place
                Pose.header.frame_id = 'map'
                Pose.pose.position.x = -0.257431030273
                Pose.pose.position.y = -1.1128064394
                Pose.pose.orientation.z = 0.581594191623
                Pose.pose.orientation.w = 0.731730386103
            else:
                print('Wrong answer, cancelling')
            self.pub.publish(Pose)

    def PublishPose(self, chosenPose):
        self.pub.publish(chosenPose)

if __name__ == '__main__':
    c = GoToDefPose()