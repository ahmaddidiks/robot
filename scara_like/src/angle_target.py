#!/usr/bin/env python3

import rospy
from scara_like.msg import target, sensor_data
from std_msgs.msg import Float32MultiArray, Empty
from geometry_msgs.msg import Point
from math import sin, cos, degrees, radians, atan2, sqrt, acos

meja = Point()
benda = Point()
present = Point()
start = False
finish = False

def target_cb(data):
    global benda, meja, start
    benda = data.benda
    meja = data.meja
    print(benda)
    start = True

def angle_cb(data):
    global present
    present.x = data.x
    present.y = data.y
    present.z = data.z

def finish_target_cb(data):
    global finish
    finish = True

def wait_finish():
    global finish
    while not finish:
        pass
    finish = False

def ik(x,y,z):
        l1, l2, l3 = 70.2, 152.7, 149.7
        try:
            tetha1 = degrees(atan2(x, y))
            # y tujuan-l1 dan x tujuan menjadi nol
            y = sqrt(x**2 + y**2) - l1
            x = 0
            # print(x,y)
            #
            A = (y**2 + x**2 - l2**2 - l3**2) /(2*l2*l3)
            tetha4 = degrees(acos(A))
            
            buff1 = l3*cos(radians(tetha4)) + l2
            buff2 = sqrt(x**2)
            buff3 = l3*sin(radians(tetha4))

            B = degrees(atan2(buff2,x))
            C = degrees(atan2(buff3,buff1))
            tetha3 = B - C
            tetha2 = z/300 * 360
            #sesuai joint
            return [tetha1, tetha2, tetha3, tetha4]
        except:
            rospy.loginfo('ik gagal')
            return [0,0,0,0]

if __name__ == "__main__":
    rospy.init_node("angle_target_publisher")
    rospy.Subscriber("ik_target", target, target_cb)
    rospy.Subscriber("data_sensor_publisher", sensor_data, angle_cb)
    rospy.Subscriber("finish_target", Empty, finish_target_cb)
    angle_target_pub = rospy.Publisher("angle_target", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if start:
            print(start)
            angle_target = Float32MultiArray()
            angle_target.data = ik(present.x, present.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(benda.x, benda.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(benda.x, benda.y, benda.z)
            angle_target_pub.publish(angle_target)
            wait_finish()

            # grip on

            angle_target.data = ik(benda.x, benda.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(meja.x, meja.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(meja.x, meja.y, meja.z)
            angle_target_pub.publish(angle_target)
            wait_finish()

            # grip off

            angle_target.data = ik(meja.x, meja.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(0, 200, 0)
            angle_target_pub.publish(angle_target)
            wait_finish()

            start = False
        
        rate.sleep()