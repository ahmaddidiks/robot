#!/usr/bin/env python3

import rospy
import math
import numpy as np
#from std_msgs.msg import UInt16
from scara.msg import stepperPostList

#length in mm
l0 = 20
l1 = 100
l2 = 100
l3 = 0

stepperPostList_pub = rospy.Publisher('set_position', stepperPostList, queue_size = 10)

def calculation(x, y, z):
        try:
            A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
            B = math.sqrt(1-(A**2))
            tetha3 = math.degrees(math.acos(A))
            
            buff1 = l2*math.cos(math.radians(tetha3)) + l1
            buff2 = math.sqrt(x**2 + y**2)
            buff3 = l2*math.sin(math.radians(tetha3))

            C = math.degrees(math.atan2(buff2,z))
            D = math.degrees(math.atan2(buff3,buff1))
            tetha2 = C - D
            tetha5 = math.degrees(math.atan2(x,y))
            tetha1 = tetha5
            tetha4 = tetha3 + tetha2 - 90
            tetha6 = rotate

            return [tetha1, tetha2, tetha3, rotate]
        
        except ValueError:
            rospy.logerr('[IK_Target] Value Error')

if __name__ == '__main__' :
    rospy.init_node('scara_invers_kinematics_node')
    
    
    rospy.Subscriber(scara_motion_control_node)
    rate = rospy.rate(60)
    rospy.spin()
