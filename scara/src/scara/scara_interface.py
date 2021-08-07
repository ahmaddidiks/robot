#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

import rospy
from geometry_msgs.msg import Point

last_traj = Point()
traj = Point()
traj_table = Point()

rospy.init_node('interface')
point_pub = rospy.Publisher('interface', Point, queue_size=10)
table_pub = rospy.Publisher('table', Point, queue_size=10)

class MainWindow(QMainWindow):
    def __init__(self, parent= None):
        super(MainWindow, self).__init__(parent)
        loadUi('/home/didik/robot/src/scara/src/scara/gui.ui', self)
        self.home.released.connect(self.home_handler)
        self.push.released.connect(self.push_handler)
        self.x_target.textChanged.connect(self.x_changed)
        self.y_target.textChanged.connect(self.y_changed)
        self.z_target.textChanged.connect(self.z_changed)
        self.x_target_2.textChanged.connect(self.x_changed_2)
        self.y_target_2.textChanged.connect(self.y_changed_2)
        self.z_target_2.textChanged.connect(self.z_changed_2)

    def home_handler(self):
        traj.x = 0
        traj.y = 170
        traj.z = 0
        point_pub.publish(traj)

    def push_handler(self):
        point_pub.publish(traj)
        table_pub.publish(traj_table)


    def x_changed(self, data):
        try:
            traj.x = float(data)
        except:
            'input a real number'
    def y_changed(self, data):
        try:
            traj.y = float(data)
        except:
            'input a real number'
    def z_changed(self, data):
        try:
            traj.z = float(data)
        except:
            'input a real number'
    def x_changed_2(self, data):
        try:
            traj_table.x = float(data)
        except:
            'input a real number'
    def y_changed_2(self, data):
        try:
            traj_table.y = float(data)
        except:
            'input a real number'
    def z_changed_2(self, data):
        try:
            traj_table.z = float(data)
        except:
            'input a real number'

app = QApplication(sys.argv)
window = MainWindow()

window.show()
app.exec_()