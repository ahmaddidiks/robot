#!/usr/bin/env python3

import rospy
from scara_like.msg import sensor_data
from scara_like.msg import encoder
from math import cos, sin, radians

sensor = sensor_data()
sensor.joints = [0,0,0,0]

rospy.init_node('data_sensor_publisher')
pub = rospy.Publisher('data_sensor_publisher', sensor_data, queue_size=10)

def forward_kinematics(tetha1, tetha2, tetha3, tetha4):
    l1, l2, l3 = 70.2, 152.7, 149.7 #mm
    tetha3 += tetha1
    tetha4 += tetha3
    y = l1*cos(radians(tetha1)) + l2*cos(radians(tetha3)) + l3*cos(radians(tetha4))
    x = l1*sin(radians(tetha1)) + l2*sin(radians(tetha3)) + l3*sin(radians(tetha3)) 
    z = float(tetha2 / 360 * 0.3)
    return x,y,z

def sensor_cb(data):
    joints = [0,0,0,0]
    tetha1, tetha2, tetha3, tinggi = data.encoderPostList
    joints = [tetha1, tinggi, tetha2, tetha3] #sudah sesuai dengan software
    joints[0] = -joints[0] * 90/2400
    joints[1] = -joints[1] * 9.257142857/2400
    joints[2] = -joints[2] * 88.76712328/2400
    joints[3] = -joints[3] * 53.114754098/2400
    sensor.joints = joints
    sensor.x, sensor.y, sensor.z = forward_kinematics(joints[0], joints[1], joints[2], joints[3]) 
    pub.publish(sensor)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.Subscriber('encoder', encoder, sensor_cb)