#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
import numpy as np
import tf
from geometry_msgs.msg import Quaternion

def getLineEquation(x1,y1,x2,y2):
    #y=ax+b
    if x1 == x2:
        a = 0
    else:
        a = float(y1-y2)/float(x1-x2)
    b=y2 - a*float(x2)
    return a,b

def getLineCirclePts(a,b,xc,yc,r):
    #y=ax+b
    #(x-xc)^2+(y-yc)^2=r^2
    A=(1+a**2)
    B=(2*a*b -2*xc-2*a*yc)
    C=(xc**2-2*b*yc+yc**2+b**2-r**2)

    #delta=beta^2 - 4*A*C

    delta=B**2 - 4*A*C
    result=[]
    if delta >0:
        r1=(-B +math.sqrt(delta))/(2*A)
        r2=(-B -math.sqrt(delta))/(2*A)

        result.append([r1,(a*r1)+b])
        result.append([r2,(a*r2)+b])
        return result
    if delta == 0:
        r1=(-B)/float(2*A)
        result.append([r1,(a*r1)+b])
        return result
    return

def ptDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)


def computeQuaternion( x0, y0, x1, y1):
    #rospy.loginfo("x0:"+str(x0)+",y0"+str(y0)+"      ,x1:"+str(x1)+",y1"+str(y1))
    y = np.arctan2(float(y1-y0),float(x1-x0))
    #y=y+math.pi
    y=y
    p = 0
    r = 0
    #print '[x0:'+str(x0)+',y0:'+str(y0)+'] ' + ' [x1:'+str(x1)+',y1:'+str(y1)+'] '
    #print 'angle: '+str(180*y/math.pi)
    quat = tf.transformations.quaternion_from_euler(r, p, y)
    quaternion = Quaternion()
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    return quaternion