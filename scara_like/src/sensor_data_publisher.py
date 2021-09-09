#!/usr/bin/env python3

import rospy
from rospy.timer import Rate
from scara_like.msg import encoder
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Path
from math import cos, degrees, sin, radians

joints = JointState()
joints.name = ['joint1', 'joint2', 'joint3', 'joint4']
joints.position = [0,0,0,0]

point = PointStamped()
point.header.frame_id = "base_link"

path = Path()
path.header.frame_id = "base_link"
path.poses = []
data_ke = 0
sudut = [0,0,0,0]

posisi = Point()

def forward_kinematics(data):
    l1, l2, l3 = 70.2, 152.7, 149.7 #mm
    tetha = [0,0,0,0]
    tetha = data
    tetha[2] += tetha[0]
    tetha[3] += tetha[2]
    y = -(l1*cos(radians(tetha[0])) + l2*cos(radians(tetha[2])) + l3*cos(radians(tetha[3])))/1000
    x = -(l1*sin(radians(tetha[0])) + l2*sin(radians(tetha[2])) + l3*sin(radians(tetha[3])))/1000
    z = tetha[1] * 0.3 / 360 + 50/1000
    return x,y,z

def sensor_cb(data):
    global sudut
    sudut = list(data.deg) #data.count format = [tetha1, tetha3, tetha4, tetha 2 sbg tinggi (z)]
    
def make_joints():
    tetha = [0,0,0,0]
    for i in range(len(sudut)):
        if i == 1:
            tetha[1] = sudut[1] * 0.3 / 360
        else:
            tetha[i] = -radians(sudut[i])
    return tetha

def make_path():
    global data_ke
    poseStamped = PoseStamped()
    poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z = forward_kinematics(sudut)
    poseStamped.header.stamp = rospy.Time.now()
    data_ke += 1
    poseStamped.header.seq = data_ke
    path.poses.append(poseStamped)

def fk(data):
    l1, l2, l3 = 70.2, 152.7, 149.7 #mm
    tetha = [0,0,0,0]
    tetha = data
    tetha[2] += tetha[0]
    tetha[3] += tetha[2]
    y = -(l1*cos(radians(tetha[0])) + l2*cos(radians(tetha[2])) + l3*cos(radians(tetha[3])))
    x = -(l1*sin(radians(tetha[0])) + l2*sin(radians(tetha[2])) + l3*sin(radians(tetha[3])))
    z = tetha[1] * 300 / 360
    return x,y,z

def make_posisi():
    posisi.x, posisi.y, posisi.z = fk(sudut)
    posisi_pub.publish(posisi)

if __name__ == '__main__':
    rospy.init_node('sensor_publisher')
    rate = Rate(40)
    joints_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    # posisi_pub = rospy.Publisher('posisi', Point, queue_size=10)
    rospy.Subscriber('sensor_data', encoder, sensor_cb)
    
    while not rospy.is_shutdown():
        joints.header.stamp = rospy.Time.now()
        path.header.stamp = rospy.Time.now()
        joints.position =  make_joints()
        joints_pub.publish(joints)
        make_path()
        path_pub.publish(path)

        #posisi 
        # make_posisi()
        rate.sleep()
    