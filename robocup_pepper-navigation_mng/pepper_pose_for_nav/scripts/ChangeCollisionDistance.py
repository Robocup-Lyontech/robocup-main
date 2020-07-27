#!/usr/bin/env python  

"""Example: Use setExternalCollisionProtectionEnabled Method"""

import qi
import argparse
import sys
import rospy


def CollisionDistance(session):
    """
    This example uses the setExternalCollisionProtectionEnabled method.
    """
    # Get the service ALMotion.

    motion_service  = session.service("ALMotion")

    motion_service.setOrthogonalSecurityDistance(0.05)
    motion_service.setTangentialSecurityDistance(0.05)
    print ("OrthogonalSecurity set to" + str(motion_service.getOrthogonalSecurityDistance()) +"," + "TangentialSecurity set to" + str(motion_service.getOrthogonalSecurityDistance()))
    
    

if __name__ == "__main__":
    rospy.init_node('pepper_collision_distance')
    ip=rospy.get_param('~ip',"127.0.0.1")
    port=rospy.get_param('~port',9559)
   
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    CollisionDistance(session)
    rospy.loginfo("Collision distance has been set to 0.05...")
