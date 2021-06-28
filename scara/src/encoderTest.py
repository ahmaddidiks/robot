#!/usr/bin/env python

'''
coding test library encoder dan stepper
'''

import rospy
from rospy.timer import Rate
from scara.msg import encoder

#inisasi node
rospy.init_node("encoder_test")
rate = rospy.Rate(10)
#pub = rospy.Publisher("encoder", encoder, queue_size=10)

def encoder_callback(data):
    for i in range(len(data)-1):
        rospy.loginfo(f"encoder {i} = ", data[i])

if __name__ == '__name__':
    while not rospy.is_shutdown():
        rospy.Subscriber("encoder", encoder, encoder_callback)
        rospy.sleep()

