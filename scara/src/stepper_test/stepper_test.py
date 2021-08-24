#!/usr/bin/env python3

import rospy
from rospy.timer import sleep
from scara_like.msg import stepper
from std_msgs.msg import Int32

import sys
from PyQt5.QtWidgets import  QMainWindow, QApplication, QDialog
from PyQt5.uic import loadUi

joints = stepper()
joints.enable = True
joints.position = [0.0,0.0,0.0,0.0]
joints.speed = [50.0, 40.0, 50.0, 50.0]

gripper = Int32()

rospy.init_node('stepper_test')
joints_pub = rospy.Publisher('real_robot', stepper, queue_size=10)
gripper_pub = rospy.Publisher('gripper', Int32, queue_size=10)

class MainUI(QMainWindow):
    def __init__(self):
        super(MainUI, self).__init__()
        loadUi('/home/didik/robot/src/scara/src/stepper_test/stepper_test.ui', self)
        self.t1.valueChanged.connect(self.t1_handler)
        self.t2.valueChanged.connect(self.t2_handler)
        self.t3.valueChanged.connect(self.t3_handler)
        self.t4.valueChanged.connect(self.t4_handler)
        self.buka.clicked.connect(self.buka_handler)
        self.tutup.clicked.connect(self.tutup_handler)

    def t1_handler(self, data):
        global joints
        joints.position[0] = float(data)
        joints_pub.publish(joints)

    def t2_handler(self, data):
        global joints
        joints.position[1] = float(data)
        joints_pub.publish(joints)

    def t3_handler(self, data):
        global joints
        joints.position[2] = float(data)
        joints_pub.publish(joints)

    def t4_handler(self, data):
        global joints
        joints.position[3] = float(data)
        joints_pub.publish(joints)

    
    def buka_handler(self):
        global gripper
        for i in range(gripper.data , 0,-1):
            gripper.data = i
            gripper_pub.publish(gripper)
            rospy.sleep(0.02)
    
    def tutup_handler(self):
        global gripper
        for i in range(gripper.data ,25):
            gripper.data = i
            gripper_pub.publish(gripper)
            rospy.sleep(0.02)

app = QApplication(sys.argv)
window = MainUI()

window.show()
app.exec_()
