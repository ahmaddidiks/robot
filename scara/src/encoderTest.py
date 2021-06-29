#!/usr/bin/env python

'''
coding test library encoder (hanya subcriber)
'''

import rospy
from rospy.timer import Rate
from scara.msg import encoder

#inisasi node
# rate = Rate(100)
#pub = rospy.Publisher("encoder", encoder, queue_size=10)

def encoder_callback(data):
    a, b, c, d, e =  data.encoderPostList
    rospy.loginfo(a)

if __name__ == '__main__':
    rospy.init_node("encoder_test")
    rospy.Subscriber("encoder", encoder, encoder_callback)
    rospy.loginfo("Init")
    rospy.spin()

