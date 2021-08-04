#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from scara.msg import stepper

from math import radians

joints = JointState()
joints.position = [0,0,0,0]
joints.name = ['joint1', 'joint2', 'joint3', 'joint4']

rospy.init_node("sensor")
rate = rospy.Rate(60)
print('test')
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

def ik_callback(data):
    tetha1, tetha2, tetha3, tinggi = data.stepperPostList
    #trasnform it into radians (coz sensor_msgs use radians not degrees)
    joint1 = radians(tetha1)
    joint2 = tinggi/360 * 0.3
    joint3 = radians(tetha2)
    joint4 = radians(tetha3)

    joints.position = [joint1,joint2, joint3, joint4]

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.Subscriber("invers_kinematics", stepper, ik_callback)
        joints.header.stamp = rospy.Time.now()
        pub.publish(joints)
        rate.sleep()