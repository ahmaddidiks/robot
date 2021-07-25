#!/usr/bin/env python3

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
    a, b, c, d =  data.encoderPostList
    rospy.loginfo(f'{a}, {b}, {c}, {d}')

if __name__ == '__main__':
    rospy.init_node("encoder_test")
    rospy.Subscriber("encoder", encoder, encoder_callback)
    rospy.loginfo("Init")
    rospy.spin()

