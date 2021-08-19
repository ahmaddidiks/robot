#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from scara_like.msg import target
from scara.msg import stepper

joints = JointState()
stepper = stepper()

init = [0,0,0,0]

def traj_cb(target):
