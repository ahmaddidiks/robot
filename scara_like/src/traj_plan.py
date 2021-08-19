#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from scara_like.msg import target
from math import sin, cos, acos, atan2, sqrt, radians, degrees

benda = Point()
meja = Point()
ik_target = target()
ik_target.benda = [0,0,0,0]
ik_target.meja = [0,0,0,0]

def ik(x,y,z):
    l1, l2, l3 = 70.2, 152.7, 149.7
    try:
        tetha1 = degrees(atan2(x, y))
        # y tujuan-l1 dan x tujuan menjadi nol
        y = sqrt(x**2 + y**2) - l1
        x = 0
        print(x,y)
        #
        A = (y**2 + x**2 - l2**2 - l3**2) /(2*l2*l3)
        tetha4 = degrees(acos(A))
        
        buff1 = l3*cos(radians(tetha3)) + l2
        buff2 = sqrt(x**2)
        buff3 = l3*sin(radians(tetha3))

        B = degrees(atan2(buff2,x))
        C = degrees(atan2(buff3,buff1))
        tetha3 = B - C
        tetha2 = z/300 * 360
        #sesuai joint
        return [tetha1, tetha2, tetha3, tetha4]
    except:
        rospy.loginfo('ik gagal')
        return [0,0,0,0]
        
def benda_cb(data):
    benda.x = data.x
    benda.y = data.y
    benda.z = data.z
    ik_target.benda = ik(benda.x, benda.y, benda.z)
    

def meja_cb(data):
    meja.x = data.x
    meja.y = data.y
    meja.z = data.z
