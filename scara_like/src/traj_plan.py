#!/usr/bin/env python3

import rospy, time
from math import log, exp
from sensor_msgs.msg import JointState
from scara_like.msg import encoder, stepper
from std_msgs.msg import Float32MultiArray, Empty

error = 0.5               #deg
init = [0, 0, 0, 0]
v_max = [50, 40, 50, 50]#[25,20,25,25]#[50, 40, 50, 50]
b = list()
c = list()
data_sensor = list()
angle_start = list()
angle_target = list()
time_start = int()

def sensor_data_handler(data):
    global data_sensor
    data_sensor = data.deg
    # print(data_sensor)

def traj_cb(data):
    global b, c, time_start, angle_start, angle_target, v_max

    time_start = time.time()
    angle_start = data_sensor
    angle_target = data.data
    
    for i in range(len(angle_target)):
        b_i, c_i = calculate_param(v_max[i], angle_start[i], angle_target[i], error)
        b.append(b_i)
        c.append(c_i)

def calculate_xt(t):
    xt = list()
    # rospy.loginfo(angle_target)
    for i in range(len(angle_target)):
        xt_i = calculate_sigmoid(v_max[i], angle_start[i], angle_target[i], t)
        xt.append(xt_i)
    return xt

def calculate_sigmoid(v_max, x0, xf, t):
    b, c = calculate_param(v_max, x0, xf, 0.001)
    xt = x0 + (xf-x0) / (1 + exp(-b * (t-c)))
    return xt

def calculate_param(v_max, x0, xf, error):
    if (xf - x0 == 0):
        b = 99999999999999999999999999999999999999
    else:
        b = 4 * abs(v_max) / abs(xf - x0)
    c = 1 / b * log((1 - error) / error)
    return b, c

def list_diff(list1, list2):
    differece = []
    zip_object = zip(list1, list2)
    for el1, el2 in zip_object:
        differece.append(abs(el1 - el2))
    return differece

if __name__ == "__main__":
    rospy.init_node("trajectory")
    rospy.Subscriber("angle_target", Float32MultiArray, traj_cb)
    rospy.Subscriber("sensor_data", encoder, sensor_data_handler)
    data_pub = rospy.Publisher("real_robot", stepper, queue_size=10)
    finish_pub = rospy.Publisher("finish_target", Empty, queue_size=10)

    data = stepper()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if angle_target != None:
            t = time.time() - time_start
            xt = calculate_xt(t)
            
            data.position = xt
            data.speed = v_max
            data.enable = True
            data_pub.publish(data)          

                       #error
            if all(i <= error for i in list_diff(angle_target, data_sensor)):
                angle_target = None
                finish_pub.publish()

        rate.sleep()
