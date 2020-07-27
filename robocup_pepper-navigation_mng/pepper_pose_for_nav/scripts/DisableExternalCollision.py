#!/usr/bin/env python  

"""Example: Use setExternalCollisionProtectionEnabled Method"""

import qi
import argparse
import sys
import rospy


class DisableExternalCollision:

    isActivatedExtColl=True
    isActivatedAutoDiagReflex=True

    def __init__(self,session,isActivatedExtColl,isActivatedAutoDiagReflex):
        self.isActivatedExtColl=isActivatedExtColl
        self.isActivatedAutoDiagReflex=isActivatedAutoDiagReflex
        self.session=session

    """
    This example uses the setExternalCollisionProtectionEnabled method.
    """
    def execute(self):
        # Get the service ALMotion.
        motion_service  = self.session.service("ALMotion")
        #value to set is "All" and "Move" deactivation require the owner consent on Pepper,"Arms", "LArm" and "RArm" deactivation does not require the owner consent.
        motion_service.setExternalCollisionProtectionEnabled("Move", self.isActivatedExtColl)

        motion_service.setDiagnosisEffectEnabled(self.isActivatedAutoDiagReflex)
        rospy.loginfo("WARNING --> isActivatedExtColl:"+str(self.isActivatedExtColl))
        rospy.loginfo("WARNING --> isActivatedAutoDiagReflex:"+str(self.isActivatedAutoDiagReflex))
        #print ("OrthogonalSecurity set to" + str(motion_service.getOrthogonalSecurityDistance()) +"," + "TangentialSecurity set to" + str(motion_service.getOrthogonalSecurityDistance()))
    
    

if __name__ == "__main__":
    rospy.init_node('pepper_disable_exernal_collision')
    ip=rospy.get_param('~ip',"192.168.0.189")
    port=rospy.get_param('~port',9559)
    isActivatedExtColl=rospy.get_param('~isActivatedExtColl',True)
    isActivatedAutoDiagReflex=rospy.get_param('~isActivatedAutoDiagReflex',True)
   
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    disableExtColl=DisableExternalCollision(session,isActivatedExtColl,isActivatedAutoDiagReflex)
    disableExtColl.execute()
    rospy.loginfo("WARNING ---> EXTERNAL COLLISION HAS BEEN processed")
