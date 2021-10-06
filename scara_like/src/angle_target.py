#!/usr/bin/env python3

import rospy
from scara_like.msg import target, encoder
from std_msgs.msg import Float32MultiArray, Empty, Int32
from geometry_msgs.msg import Point
from math import sin, cos, degrees, radians, atan2, sqrt, acos
from time import sleep

import csv

meja = Point()
benda = Point()
present = Point()
start = False
finish = False
pos = [0,0,0,0]
aktual_tetha = [0,0,0,0]

def target_cb(data):
    global benda, meja, start
    benda = data.benda
    meja = data.meja
    print(benda)
    start = True

def angle_cb(data):
    global pos, present, aktual_tetha
    pos = list(data.deg).copy()
    aktual_tetha = pos.copy()
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


def csv_save(iter):
    data = [iter]
    target_tetha = list(angle_target.data).copy()

    for i in range(len(aktual_tetha)):
        aktual_tetha[i] = round(aktual_tetha[i], 4)
        target_tetha[i] = round(target_tetha[i], 4)

    # target_posisi = [benda.x, benda.y, benda.z]
    target_posisi = [benda.x, benda.y, benda.z]
    aktual_posisi = [present.x, present.y, present.z*1000]
    for i in range(len(aktual_posisi)):
        
        target_posisi[i] = round(target_posisi[i], 4)
        aktual_posisi[i] = round(aktual_posisi[i], 4)

    data = data + target_tetha + aktual_tetha + target_posisi + aktual_posisi
    # print(data)

    writer.writerow(data)
 
def pengujian_lingkaran(centerX=0, centerY=310, radius=60, step=5):
    '''
    rumus lingkaran = (x-h)^2 + (y-k)^2 = r^2
                    = (x-h)^2 = r^2 - (y-k)^2
    dengan h=centerX dan k=centerY
    ''' 
    x=0
    iter = 0
    for y in range(centerY-radius, centerY+radius+step, step):
        x = sqrt(radius**2 - (y-centerY)**2) #masih x-centerX
        x +=centerX
        # print(x,y)
        angle_target.data = ik(x, y, 0)
        angle_target_pub.publish(angle_target)
        wait_finish()
        iter+=1
        benda.x, benda.y, benda.z = x,y,0 #hanya untuk menyimpan target posisi
        csv_save(iter)
        sleep(0.1)
    
    #kembali ke titik awal
    for y in range(centerY+radius, centerY-radius-step, -step):
        x = sqrt(radius**2 - (y-centerY)**2) #masih x-centerX
        x +=centerX
        x=-x
        # print(x,y)
        angle_target.data = ik(x, y, 0)
        angle_target_pub.publish(angle_target)
        benda.x, benda.y, benda.z = x,y,0 #hanya untuk menyimpan target posisi
        wait_finish()
        iter+=1
        csv_save(iter)
        sleep(0.1)

def pengujian_diagonal():
    iter=0
    xStart=0
    yStart=360
    zStart=0
    xFinish=360
    yFinish=0
    zFinish=100
    sampling=30

    for i in range(1, sampling+1, 1):
        benda.x = xStart + ((xFinish-xStart)/sampling) * i
        benda.y = yStart - abs((yFinish-yStart)/sampling) * i
        benda.z = zStart + ((zFinish-zStart)/sampling) * i
        angle_target.data = ik(benda.x, benda.y, benda.z)
        angle_target_pub.publish(angle_target)
        wait_finish()
        iter+=1
        csv_save(iter)
        sleep(0.1)

    



def pengujian_pick_and_place():
    angle_target.data = ik(0, 270, 0)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    
    angle_target.data = ik(present.x, present.y, benda.z+15)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)

    rospy.loginfo('menuju sub 1 benda')
    angle_target.data = ik(benda.x, benda.y, benda.z+15)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    rospy.loginfo('menuju benda')
    angle_target.data = ik(benda.x, benda.y, benda.z)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    rospy.loginfo('gripper on')
    gripper.data = 20
    gripper_pub.publish(gripper)
    rospy.sleep(0.02)

    rospy.loginfo('menuju sub meja 1')
    angle_target.data = ik(benda.x, benda.y, benda.z+80)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    rospy.loginfo('menuju sub meja 2')
    angle_target.data = ik(meja.x, meja.y, meja.z+80)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    rospy.loginfo('menuju meja')
    angle_target.data = ik(meja.x, meja.y, meja.z)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    rospy.loginfo('gripper off')
    # grip off
    # for i in range(5, 20):
    #     gripper.data -= i
    #     gripper_pub.publish(gripper)
    #     rospy.sleep(0.02)
    gripper.data = 5
    gripper_pub.publish(gripper)
    rospy.sleep(0.02)

    angle_target.data = ik(meja.x, meja.y, meja.z+80)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)
    # csv_save(percobaan)

    angle_target.data = ik(0, 270, 0)
    angle_target_pub.publish(angle_target)
    wait_finish()
    rospy.loginfo(present)

if __name__ == "__main__":
    rospy.init_node("angle_target_publisher")
    rospy.Subscriber("ik_target", target, target_cb)
    rospy.Subscriber("sensor_data", encoder, angle_cb)
    rospy.Subscriber("finish_target", Empty, finish_target_cb)
    angle_target_pub = rospy.Publisher("angle_target", Float32MultiArray, queue_size=10)
    gripper_pub = rospy.Publisher("gripper", Int32, queue_size=10)

    gripper = Int32()

    rate = rospy.Rate(2)

    '''save data'''
    f = open('/home/didik/robot/src/scara_like/data_percobaan/data_diagonal.csv', 'w')
    writer = csv.writer(f)
    header = ['percobaan', 'target tetha 1', 'target tetha 2', 'target tetha 3', 'target tetha 4',\
                            'aktual tetha 1', 'aktual tetha 2', 'aktual tetha 3', 'aktual tetha 4',\
                            'target pos x', 'target pos y', 'target pos z',\
                            'aktual pos x', 'aktual pos y', 'aktual pos z']
    writer.writerow(header)
    # percobaan = 0
    # last = 0
    while not rospy.is_shutdown():
        if start:
            # rospy.loginfo('mulai')
            angle_target = Float32MultiArray()
            # for i in range(245, 370, 5):
                
            #     percobaan += 1
            #     # rospy.loginfo('menuju sub 1 benda')
            # angle_target.data = ik(benda.x, benda.y, benda.z)
            # # last = rospy.get_time()
            # angle_target_pub.publish(angle_target)
            # wait_finish()
                # print(rospy.get_time() - last)
                
                # csv_save(percobaan)
                # benda.z += 5
            # pengujian_lingkaran()
            pengujian_diagonal()
            
            # csv_save(percobaan)
            start = False
            # percobaan +=1
        
        rate.sleep()

