#!/usr/bin/env python3

import rospy
from scara_like.msg import sensor_data
from scara_like.msg import encoder
from sensor_msgs.msg import JointState
from math import cos, sin, radians

sensor = sensor_data()
sensor.joints = [0,0,0,0]
joints = JointState()

rospy.init_node('data_sensor_publisher')
pub = rospy.Publisher('data_sensor_publisher', sensor_data, queue_size=10)
joints_pub = rospy.Publisher("joints_state_publisher", JointState, queue_size=10)

def forward_kinematics(tetha1, tetha2, tetha3, tetha4):
    l1, l2, l3 = 70.2, 152.7, 149.7 #mm
    tetha3 += tetha1
    tetha4 += tetha3
    y = l1*cos(radians(tetha1)) + l2*cos(radians(tetha3)) + l3*cos(radians(tetha4))
    x = l1*sin(radians(tetha1)) + l2*sin(radians(tetha3)) + l3*sin(radians(tetha3)) 
    z = float(tetha2 / 360 * 0.3)
    return x,y,z

def sensor_cb(data):
    tetha = [0,0,0,0]
    tetha = data.count #data.count format = [tetha1, tetha3, tetha4, tetha 2 sbg tinggi (z)]
    
    tetha[0] = -tetha[0] * 90/2400
    tetha[1] = -tetha[1] * 9.257142857/2400
    tetha[2] = -tetha[2] * 88.76712328/2400
    tetha[3] = -tetha[3] * 53.114754098/2400
    sensor.joints = tetha
    sensor.x, sensor.y, sensor.z = forward_kinematics(tetha[0], tetha[1], tetha[2], tetha[3]) 
    
    joints.position = sensor.joints
    joints.position[1] = joints.position[1] / 360 * 0.3 #convert dari derajat ke tinggi 0 - 0.3m  
    pub.publish(sensor)
    joints.header.stamp = rospy.Time.now()
    joints_pub.publish(joints)
    



if __name__ == '__main__':
    rospy.Subscriber('encoder', encoder, sensor_cb)
    rospy.spin()
    