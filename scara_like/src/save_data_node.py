#!/usr/bin/env python3

from os import write
import rospy
from rospy.core import is_shutdown
from scara_like.msg import encoder, stepper, posisi, speed, acc, compare

from angle_target import forward_kinematics
import csv

# real_robot = list()
sensor = list()
data = compare()

counter = 0
last = 0

data.kec_robot = [0,0,0,0]
data.kec_target = [0,0,0,0]
data.a_target = [0,0,0,0]
data.a_robot = [0,0,0,0]

last_pos_robot = [0,0,0,0]
last_pos_target = [0,0,0,0]
last_kecepatan_robot = [0,0,0,0]
last_kecepatan_target = [0,0,0,0]

pos = posisi()
vel = speed()
# a = acc()

x1_last = 0
y1_last = 0
z1_last = 0

x2_last = 0
y2_last = 0
z2_last = 0

vx1_last = 0
vy1_last = 0
vz1_last = 0

vx2_last = 0
vy2_last = 0
vz2_last = 0

def sensor_cb(data):
    global sensor
    sensor = list(data.deg).copy()

def real_robot_cb(derajat):
    global counter, last
    global x1_last, y1_last, z1_last,\
           x2_last, y2_last, z2_last,\
           vx1_last, vx2_last, vy1_last,\
           vy2_last, vz1_last, vz2_last
    global last_kecepatan_robot, last_kecepatan_target,\
            last_pos_robot, last_pos_target
    
    #posisi (sudut)
    data.der_target = list(derajat.position).copy()
    data.der_robot = sensor.copy()

    for i in range(len(sensor)):
        #kecepatan (derajat/sec)    
        data.kec_robot[i] = (data.der_robot[i] - last_pos_robot[i]) * 100/16.666
        data.kec_target[i] = (data.der_target[i] - last_pos_target[i])* 100/16.666
        
        #akselerasi
        data.a_robot[i] = (data.kec_robot[i] - last_kecepatan_robot[i]) /16.666
        data.a_target[i] = (data.a_target[i] - last_kecepatan_target[i]) /16.666

        #4 angka belakang koma
        data.kec_robot[i] = round(data.kec_robot[i], 4)
        data.kec_target[i] = round(data.kec_target[i], 4)
        data.a_robot[i] = round(data.a_robot[i], 4)
        data.a_target[i] = round(data.a_target[i], 4)

    
    # simpan = data.der_target + data.der_robot + data.kec_target + data.kec_robot + data.a_target + data.a_robot
    # simpan = [data.der_target[1], data.der_robot[1], data.kec_target[1], data.kec_robot[1], data.a_target[1], data.a_robot[1]]
    # if counter == 0:
    #     last = rospy.get_time()
    #     simpan = [0]
    #     counter = 1
    # else:
    #     simpan = [rospy.get_time() - last]
    # writer.writerow(simpan)

    #copy for last pos
    last_kecepatan_target = data.kec_target.copy()
    last_kecepatan_robot = data.kec_robot.copy()
    last_pos_robot = data.der_robot.copy()
    last_pos_target = data.der_target.copy()

    real_robot = list(data.der_robot).copy()
    x1, y1, z1 = forward_kinematics(real_robot)
    z1 = z1*1000
    x2, y2, z2 = forward_kinematics(sensor)
    z2 = z2*1000

    pos.target.x = x1
    pos.target.y = y1
    pos.target.z = z1

    pos.robot.x = x2
    pos.robot.y = y2
    pos.robot.z = z2

    vel.target.x = abs(x1 - x1_last) * 100 / 16.666
    vel.target.y = abs(y1 - y1_last) * 100 / 16.666
    vel.target.z = abs(z1 - z1_last) * 100 / 16.666

    vel.robot.x = abs(x2 - x2_last) * 100 / 16.666
    vel.robot.y = abs(y2 - y2_last) * 100 / 16.666
    vel.robot.z = abs(z2 - z2_last) * 100 / 16.666

    simpan = [pos.target.x, pos.robot.x, pos.target.y, pos.robot.y, pos.target.z, pos.robot.z,\
            vel.target.x, vel.robot.x, vel.target.y, vel.robot.y, vel.target.z, vel.robot.z]
    writer.writerow(simpan)
    # a.target.x = (vel.target.x - vx1_last) / 16.666 
    # a.target.y = (vel.target.y - vy1_last) / 16.666 
    # a.target.y = (vel.target.z - vz1_last) / 16.666 

    # a.robot.x = (vel.robot.x - vx2_last) / 16.666 
    # a.robot.y = (vel.robot.y - vy2_last) / 16.666 
    # a.robot.y = (vel.robot.z - vz2_last) / 16.666

    x1_last = x1
    y1_last = y1
    z1_last = z1

    x2_last = x2
    y2_last = y2
    z2_last = z2

    vx1_last = vel.target.x
    vy1_last = vel.target.y
    vz1_last = vel.target.z

    vx2_last = vel.robot.x
    vy2_last = vel.robot.y
    vz2_last = vel.robot.z

    try:
        pub.publish(data)
        pos_pub.publish(pos)
        speed_pub.publish(vel)
        # acc_pub.publish(a)
        # rospy.loginfo('berhasil save')
    except:
        rospy.loginfo('gagal save')
    
if __name__ == '__main__':
    rospy.init_node("saver")
    rospy.Subscriber("real_robot", stepper, real_robot_cb)
    rospy.Subscriber("sensor_data", encoder, sensor_cb)
    pos_pub = rospy.Publisher('compare_posisi', posisi, queue_size=10)
    speed_pub = rospy.Publisher('compare_speed', speed, queue_size=10)
    # acc_pub = rospy.Publisher('compare_acc', acc, queue_size=10)
    pub = rospy.Publisher('data_saver', compare, queue_size=10)
    rate = rospy.Rate(60)

    f = open('/home/didik/robot/src/scara_like/data_percobaan/data_posisi.csv', 'w')
    writer = csv.writer(f)
    header = ['pos.target.x', 'pos.robot.x', 'pos.target.y', 'pos.robot.y', 'pos.target.z', 'pos.robot.z',\
            'vel.target.x', 'vel.robot.x', 'vel.target.y', 'vel.robot.y', 'vel.target.z', 'vel.robot.z']
    # header = [data.der_target + data.der_robot + data.kec_target + data.kec_robot + data.a_target + data.a_robot]
    # header = ['target tetha 1', 'target tetha 2', 'target tetha 3', 'target tetha 4',\
    #         'robot tetha 1', 'robot tetha 2', 'robot tetha 3', 'robot tetha 4',\
    #         'target kecepatan tetha1', 'target kecepatan tetha2', 'target kecepatan tetha3', 'target kecepatan tetha4',\
    #         'robot kecepatan tetha 1', 'robot kecepatan tetha 2', 'robot kecepatan tetha 3', 'robot kecepatan tetha 4',\
    #         'target percepatan tetha 1', 'target percepatan tetha 2', 'target percepatan tetha 3', 'target percepatan tetha 4',\
    #         'robot percepatan tetha 1', 'robot percepatan tetha 2', 'robot percepatan tetha 3', 'robot percepatan tetha 4']
    # # header = ['target tetha2', 'robot tetha 2', 'target kecepatan tetha 2', 'robot keepatan tetha 2', 'target akselerasi tetha 2', 'robot percepatan tetha 2']
    # header = ['time']
    writer.writerow(header)

    while not rospy.is_shutdown():
        pass
        rate.sleep()

        


