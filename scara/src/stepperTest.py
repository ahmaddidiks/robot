#!/usr/bin/env python

'''
coding test fungsional stepper (hanya publish)
'''

import rospy
from rospy.timer import Rate
from scara.msg import stepper

#global var
data = stepper()
data.enableStepper = False
data.stepperPostList = [0,0,0,0,0]

#inisasi node
rospy.init_node("stepper_test")
rate = Rate(2000)
pub = rospy.Publisher("stepper_test", stepper, queue_size=10)

def streamer():
    data.enableStepper ^= True #toogle value
    for iter in range(5):
        data.stepperPostList[iter] ^ 1
    pub.publish(data)
    rospy.loginfo(data)


if __name__ == '__main__':
    rospy.loginfo("Init")
    while not rospy.is_shutdown():
        streamer()
        rate.sleep()
        
        

