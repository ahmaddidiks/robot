#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
# from scara.msg import encoder

from math import radians

sensor = JointState()
a = radians(30)
b = 0.2
c = radians(30)
d = radians(40)
sensor.position = [a,b,c,d]
# sensor.name = ['joint1', 'joint2', 'joint3', 'joint4']
# sensor.position = [0.0, 0.0, 0.0, 0.0]

rospy.init_node("sensor")
rate = rospy.Rate(60)
print('test')
pub = rospy.Publisher('sensor', JointState, queue_size=10)

# def sensor_callback(data):
#     tetha1, tetha2, tetha3, tinggi = data.encoderPostList
#     #transform into degrees from raw data sensor
#     tetha1 = -tetha1 * 90 /2400
#     tetha2 = -tetha2 * 88.76712328/2400
#     tetha3 = -tetha3 * 53.114754098/2400
#     tinggi = -tinggi * 9.257142857 / 2400
    
#     #trasnform it into radians (coz sensor_msgs use radians not degrees)
#     joint1 = radians(tetha1)
#     joint2 = tinggi/360 * 0.3
#     joint3 = radians(tetha2)
#     joint4 = radians(tetha3)

#     sensor.position = [joint1,joint2, joint3, joint4]

#     pub.publish(sensor)
#     # print(sensor)

# def sensor(data):
#     print(data.position)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        # rospy.Subscriber("joint_states", JointState, sensor)
        pub.publish(sensor)
        rate.sleep()