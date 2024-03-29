#!/usr/bin/env python3

'''
coding test invers kineamtiks dengan GUI
'''

from math import sin, cos, acos, atan2, sqrt, radians, degrees

import rospy
from rospy.timer import Rate
from scara.msg import stepper, encoder, syncEncoder

data = stepper()
data.enableStepper = False
data.stepperPostList = [0,0,0,0]

enc = syncEncoder()
enc.tetha = [0,0,0,0]

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

class MainWindows(QMainWindow):
    def __init__(self, parent= None):
        super(MainWindows, self).__init__(parent)
        loadUi('/home/didik/robot/src/scara/src/ik_test/gui.ui',self)
        title = "Manipulator GUI"
        '''coordinat pos'''
        global x_pos, y_pos, z_pos
        x_pos, y_pos, z_pos = 0.0, 0.0, 0.0

        self.stepper_btn.clicked.connect(self.enable_stepper)
        self.x_target_box.textChanged.connect(self.x_target_changed)
        self.y_target_box.textChanged.connect(self.y_target_changed)
        self.z_target_box.textChanged.connect(self.z_target_changed)
        self.run_ik_btn.clicked.connect(self.run_ik)
        self.sync_encoder.clicked.connect(self.sync_encoder_handler)

        '''subcribe from sensor data and show it in GUI'''
        self.sub_sensor = rospy.Subscriber("encoder", encoder, self.encoder_callback)

    def x_target_changed(self, value):
        global x_pos
        try:
            x_pos = float(value)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def y_target_changed(self, value):
        global y_pos
        try:
            y_pos = float(value)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def z_target_changed(self, value):
        global z_pos
        try:
            z_pos = float(value)
            print(z_pos)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def run_ik(self):
        x,y,z = x_pos, y_pos, z_pos
        print(x,y,z)
        l1, l2, l3 = 70.2, 152.7, 149.7
        high = 1
        try:
            tetha1 = degrees(atan2(x, y))
            # y tujuan-l1 dan x tujuan menjadi nol
            y = sqrt(x**2 + y**2) - l1
            x = 0
            print(x,y)
            #
            A = (y**2 + x**2 - l2**2 - l3**2) /(2*l2*l3)
            tetha3 = degrees(acos(A))
            
            buff1 = l3*cos(radians(tetha3)) + l2
            buff2 = sqrt(x**2)
            buff3 = l3*sin(radians(tetha3))

            B = degrees(atan2(buff2,x))
            C = degrees(atan2(buff3,buff1))
            tetha2 = B - C
            #z = z #nanti di kali gain / di konvert ke scala / derajat
            
            data.stepperPostList[0] = tetha1
            data.stepperPostList[1] = tetha2
            data.stepperPostList[2] = tetha3
            data.stepperPostList[3] = z

            self.tetha1_target_label.setText(str(tetha1))
            self.tetha2_target_label.setText(str(tetha2))
            self.tetha3_target_label.setText(str(tetha3))
            self.tinggi_target_label.setText(str(z))
            #return [tetha1, tetha2, tetha3, z]
            self.publish()

        except ValueError:
            rospy.loginfo('IK Value Out of Reach')
        
    def enable_stepper(self, checked):
        if(checked):
            self.stepper_btn.setText("STEPPER ON")
            data.enableStepper = True
        else:
            self.stepper_btn.setText("STEPPER OFF")
            data.enableStepper = False
        self.publish()
    
    def publish(self):
        pub.publish(data)

    def encoder_callback(self,data):
        tetha1, tetha2, tetha3, tinggi = data.encoderPostList
        tetha1 = -tetha1 * 90 /2400
        tetha2 = -tetha2 * 88.76712328/2400
        tetha3 = -tetha3 * 53.114754098/2400
        self.tetha1_real_label.setText(str(tetha1))
        self.tetha2_real_label.setText(str(tetha2))
        self.tetha3_real_label.setText(str(tetha3))
        self.tinggi_real_label.setText(str(tetha3))

rospy.init_node('invers_kinematics_gui')
pub = rospy.Publisher('invers_kinematics_gui', stepper, queue_size=10)
enc_pub = rospy.Publisher('sync_encoder',syncEncoder, queue_size=10)

app = QApplication(sys.argv)
window = MainWindows()

window.show()
app.exec_()