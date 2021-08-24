#!/usr/bin/env python3

import rospy
from scara_like.msg import target, encoder
from std_msgs.msg import Float32MultiArray, Empty, Int32
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
    pos = list(data.deg)
    present.x, present.y, present.z = forward_kinematics(pos)

def forward_kinematics(data):
    l1, l2, l3 = 70.2, 152.7, 149.7 #mm
    tetha = [0,0,0,0]
    tetha = data
    tetha[2] += tetha[0]
    tetha[3] += tetha[2]
    y = l1*cos(radians(tetha[0])) + l2*cos(radians(tetha[2])) + l3*cos(radians(tetha[3]))
    x = l1*sin(radians(tetha[0])) + l2*sin(radians(tetha[2])) + l3*sin(radians(tetha[3])) 
    z = float(tetha[1] * 0.3 / 360)
    return x,y,z

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
    rospy.Subscriber("sensor_data", encoder, angle_cb)
    rospy.Subscriber("finish_target", Empty, finish_target_cb)
    angle_target_pub = rospy.Publisher("angle_target", Float32MultiArray, queue_size=10)
    gripper_pub = rospy.Publisher("gripper", Int32, queue_size=10)

    gripper = Int32()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if start:
            # print(start)
            rospy.loginfo('menuju sub 1 benda')
            angle_target = Float32MultiArray()
            angle_target.data = ik(present.x, present.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()
            
            rospy.loginfo('menuju sub 2 benda')
            angle_target.data = ik(benda.x, benda.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            rospy.loginfo('menuju benda')
            angle_target.data = ik(benda.x, benda.y, benda.z)
            angle_target_pub.publish(angle_target)
            wait_finish()

            rospy.loginfo('gripper on')
            # grip on
            # for i in range(5,20):
            #     gripper.data = i
            #     gripper_pub.publish(gripper)
            #     rospy.sleep(0.02)
            gripper.data = 20
            gripper_pub.publish(gripper)
            rospy.sleep(0.02)

            rospy.loginfo('menuju sub meja 1')
            angle_target.data = ik(benda.x, benda.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            rospy.loginfo('menuju sub meja 2')
            angle_target.data = ik(meja.x, meja.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            rospy.loginfo('menuju meja')
            angle_target.data = ik(meja.x, meja.y, meja.z)
            angle_target_pub.publish(angle_target)
            wait_finish()

            rospy.loginfo('gripper off')
            # grip off
            # for i in range(5, 20):
            #     gripper.data -= i
            #     gripper_pub.publish(gripper)
            #     rospy.sleep(0.02)
            gripper.data = 5
            gripper_pub.publish(gripper)
            rospy.sleep(0.02)

            angle_target.data = ik(meja.x, meja.y, 100)
            angle_target_pub.publish(angle_target)
            wait_finish()

            angle_target.data = ik(0, 270, 0)
            angle_target_pub.publish(angle_target)
            wait_finish()

            start = False
        
        rate.sleep()