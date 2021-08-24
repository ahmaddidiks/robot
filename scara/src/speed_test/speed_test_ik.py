#!/usr/bin/env python3

from sys import path
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from math import degrees, radians, acos, sqrt, atan2, cos, sin

joints = JointState()
joints.position = [0,0,0,0]
joints.name = ['joint1', 'joint2', 'joint3', 'joint4']

point = PointStamped()
point.header.frame_id = "base_link"

path = Path()
path.header.frame_id = "base_link"
path.poses = []
pos_x, pos_y, pos_z = 0,0,0
count = 0

last_x, last_y, last_z = 0,0,0
# stamped = []

rospy.init_node("sensor")
rate = rospy.Rate(60)

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
pub_point = rospy.Publisher('path', Path, queue_size=10)

def forward_kinematics():
    tetha1, tinggi, tetha2, tetha3 = joints.position
    tetha1 = radians(tetha1)
    tetha2 = radians(tetha2)
    tetha3 = radians(tetha3)
    a, b, c = tetha1, tetha2, tetha3
    b += a
    c += b
    #robot link
    l1, l2, l3 = 70.2, 152.7, 149.7
    #forward kineamtics
    hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
    hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
    hasil_z = tinggi
    return hasil_x, hasil_y, hasil_z

def invers_kinematics(x,y,z):

        l1, l2, l3 = 70.2, 152.7, 149.7
        try:
            tetha1 = degrees(atan2(x, y))
            # y tujuan-l1 dan x tujuan menjadi nol
            y = sqrt(x**2 + y**2) - l1
            x = 0
            # print(x,y)
            #
            A = (y**2 + x**2 - l2**2 - l3**2) /(2*l2*l3)
            tetha3 = degrees(acos(A))
            
            buff1 = l3*cos(radians(tetha3)) + l2
            buff2 = sqrt(x**2)
            buff3 = l3*sin(radians(tetha3))

            B = degrees(atan2(buff2,x))
            C = degrees(atan2(buff3,buff1))
            tetha2 = B - C
            tinggi = z

            tetha1 = radians(tetha1)
            tetha2 = radians(tetha2)
            tetha3 = radians(tetha3)
            #z = z #nanti di kali gain / di konvert ke scala / derajat
            # a, b, c = tetha1, tetha2, tetha3
            # b += a
            # c += b
            # hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
            # hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
            # hasil_z = z
            return [tetha1, tinggi/1000, tetha2, tetha3]
        except ValueError:
            rospy.loginfo('IK Value Out of Reach')
            return Quaternion().position

def ik_callback(quaternion):
    global pos_x, pos_y, pos_z
    pos_x, pos_y, pos_z = quaternion.x, quaternion.y, quaternion.z
    if quaternion.x != last_x or quaternion.y !=  last_y or quaternion.z != last_z:
        try:
            joints.position = invers_kinematics(quaternion.x, quaternion.y, quaternion.z)
        except:
            rospy.loginfo('ik gagal')

def make_path():
    # if quater.x != last_x or quater.y !=  last_y or quater.z != last_z:
    # stamped = quater.x, quater.y, quater.z, quater.w
    global count
    poseStamped = PoseStamped()
    poseStamped.pose.position.x = pos_x/1000
    if count == 0:
        poseStamped.pose.position.y = -372.599999/1000
    poseStamped.pose.position.y = -pos_y/1000
    poseStamped.pose.position.z = (pos_z + 50)/1000 
    poseStamped.header.stamp = rospy.Time.now()
    poseStamped.header.seq = count
    count +=1
    path.poses.append(poseStamped)
    

while not rospy.is_shutdown():
    rospy.Subscriber('position', Quaternion, ik_callback)
    joints.header.stamp = rospy.Time.now()

    path.header.stamp = rospy.Time.now()
    rate.sleep()
    point.point.x, point.point.y, point.point.z = forward_kinematics()
    make_path()
    pub.publish(joints)
    pub_point.publish(path)
    rate.sleep()