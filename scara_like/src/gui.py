#!/usr/bin/env python3

import rospy
from scara_like.msg import target

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

ik_target = target()

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
    
    def publish(self):
        ik_target.benda.x = benda_x
        ik_target.benda.y = benda_y
        ik_target.benda.z = benda_z
        ik_target.meja.x = meja_x
        ik_target.meja.y = meja_y
        ik_target.meja.z = meja_z
        pub.publish(ik_target)

app = QApplication(sys.argv)
window = MainWindows()

window.show()
app.exec_()