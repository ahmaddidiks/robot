#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
from math import sin, cos, radians, degrees

import rospy
from sensor_msgs.msg import JointState
from scara.msg import encoder
rospy.init_node("results_display_value")

class MainWindows(QMainWindow):
    def __init__(self, parent: None):
        super(MainWindows, self).__intit__(parent)
        loadUi('/home/didik/robot/src/scara/src/full1/result.ui', self)
        self.sub_sensor = rospy.Subscriber('encoder', encoder, self.encoder_callback)
        self.sub_joints = rospy.Subscriber('joint_states', JointState, joints_callback)
    
    def encoder_callback(self, sensor):
        tetha1, tetha2, tetha3, tinggi = sensor.encoderPostList
        tetha1 = -tetha1 * 90 /2400
        tetha2 = -tetha2 * 88.76712328/2400
        tetha3 = -tetha3 * 53.114754098/2400
        tinggi = -tinggi * 9.257142857/2400
        
        x, y, z = self.forward_kineamtics(tetha1, tetha2, tetha3, tinggi)
        
        self.x_h.setText(str(x))
        self.y_h.setText(str(y))
        self.z_h.setText(str(z))

    def joints_callback(self, joints):
        joint1, joint2, joint3, joint4 = joints.position
        joint1 = degrees(joint1)
        tinggi = joint2
        joint2 = joint3
        joint3 = joint4

        x, y, z = self.forward_kineamtics(joint1, joint2, joint3, tinggi)
        self.x_s.setText(str(x))
        self.y_s.setText(str(y))
        self.z_s.setText(str(z))

    def forward_kineamtics(self, tetha1, tetha2, tetha3, tinggi):
        a, b, c = tetha1, tetha2, tetha3
        b += a
        c += b
        #robot link
        l1, l2, l3 = 70.2, 152.7, 149.7
        #forward kineamtics
        hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
        hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
        hasil_z = tinggi * 300 /360
        return hasil_x, hasil_y, hasil_z

app = QApplication(sys.argv)
window = MainWindows()

window.show()
app.exec_()

