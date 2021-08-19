#!/usr/bin/env python3

import rospy
from scara_like.msg import target
from math import sin, cos, acos, atan2, sqrt, radians, degrees

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

ik_target = target()
ik_target.benda = [0,0,0,0]
ik_target.meja = [0,0,0,0]

#initial benda geometry in mm (cartesian)
benda_x = 0
benda_y = 250
benda_z = 0

#initial meja geometry in mm (cartesian)
meja_x = 0
meja_y = -250
meja_z = 0

rospy.init_node('gui')
pub = rospy.Publisher('ik_target', target, queue_size=10)


class MainWindows(QMainWindow):
    def __init__(self, parent= None):
        super(MainWindows, self).__init__(parent)
        loadUi('/home/didik/robot/src/scara_like/src/gui.ui',self)
        # title = "Manipulator GUI"

        self.x_benda.textEdited.connect(self.x_benda_handler)
        self.y_benda.textEdited.connect(self.y_benda_handler)
        self.z_benda.textEdited.connect(self.z_benda_handler)

        self.x_meja.textEdited.connect(self.x_meja_handler)
        self.y_meja.textEdited.connect(self.y_meja_handler)
        self.z_meja.textEdited.connect(self.z_meja_handler)

        self.run.clicked.connect(self.publish)
    def x_benda_handler(self, data):
        global benda_x
        try:
            benda_x = int(data)
        except:
            rospy.loginfo('benda.x salah')
            benda_x = 0
    def y_benda_handler(self, data):
        global benda_y
        try:
            benda_y = int(data)
        except:
            rospy.loginfo('benda.y salah')
            benda_y = 250
    def z_benda_handler(self, data):
        global benda_z
        try:
            benda_z = int(data)
        except:
            rospy.loginfo('benda.z salah')
            benda_z = 0
    
    def x_meja_handler(self, data):
        global meja_x
        try:
            meja_x = int(data)
        except:
            rospy.loginfo('meja.x salah')
            meja_x = 0
    def y_meja_handler(self, data):
        global meja_y
        try:
            meja_y = int(data)
        except:
            rospy.loginfo('meja.y salah')
            meja_y = -250
    def z_meja_handler(self, data):
        global meja_z
        try:
            meja_z = int(data)
        except:
            rospy.loginfo('meja.z salah')
            meja_z = 0
    
    def ik(self, x,y,z):
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

    def publish(self):
        ik_target.benda = self.ik(benda_x, benda_y, benda_z)
        ik_target.meja = self.ik(meja_x, meja_y, meja_z)
        pub.publish(ik_target)

app = QApplication(sys.argv)
window = MainWindows()

window.show()
app.exec_()