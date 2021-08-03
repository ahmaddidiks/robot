#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from scara.msg import stepper

from math import degrees

motor = stepper()
motor.enableStepper = True
motor.stepperPostList = [0,0,0,0]

rospy.init_node("stepper_driver")
rate = rospy.Rate(100)
print('test')
pub = rospy.Publisher('stepper_driver', stepper, queue_size=10)

def joints_callback(joints):
    tetha1, tinggi, tetha2, tetha3 = joints.position
    #transform from radians into degrees
    tetha1 = degrees(tetha1)
    tetha2 = degrees(tetha2)
    tetha3 = degrees(tetha3)
    tinggi = tinggi * 360 / 0.3 #max 0.3m
    
    motor.stepperPostList[0] = tetha1
    motor.stepperPostList[1] = -tetha2
    motor.stepperPostList[2] = -tetha3
    motor.stepperPostList[3] = -tinggi
    #transform from degrees into step counts
    # motor.stepperPostList[0] = -int(tetha1 * 720 / 360)
    # motor.stepperPostList[1] = int(tetha2 * 730 / 360)
    # motor.stepperPostList[2] = int(tetha3 * 1220 / 360)
    # motor.stepperPostList[3] = -int(tinggi * 7000 / 360)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.Subscriber("joint_states", JointState, joints_callback)
        pub.publish(motor)
        rate.sleep()