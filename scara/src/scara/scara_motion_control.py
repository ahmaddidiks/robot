#!/usr/bin/env python3

from operator import le
import rospy
from geometry_msgs.msg import Point
import numpy as np

benda = Point()
meja = Point()
gerak = Point()
count = 0
rospy.init_node("motion_control")
rate = rospy.Rate(60)
pub = rospy.Publisher('koordinat', Point, queue_size=10)
def point_callback(traj):
    global count
    benda.x, benda.y, benda.z = traj.x, traj.y, traj.z

    x_awal , y_awal , z_awal = 0.0, 170.0, 0.0
    X = np.linspace(x_awal, benda.x, 10)
    Y = np.linspace(y_awal, benda.y, 10)
    Z = np.linspace(z_awal, benda.z, 10)

    while count < 10:
        gerak.x = X[count]
        gerak.y = Y[count]
        gerak.z = Z[count]
        pub.publish(gerak)
        count +=1
        print(count)
            
def meja_callback(traj):
    meja.x, meja.y, meja.z = traj.x, traj.y, traj.z

while not rospy.is_shutdown():
    rospy.Subscriber('interface', Point, point_callback)
    rospy.Subscriber('table', Point, meja_callback)
    rate.sleep()