import rospy
import math
import numpy as np
from std_msgs.msg import UInt16
from scara.msg import IK_Target

rospy.init_node('scara_motion_control_node', IK_Target, queue_size = 10)




if __name__ == '__name__':
    #subcribe 
    rate = rospy.rate(60)
    rospy.spin()


