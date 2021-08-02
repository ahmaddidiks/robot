#!/usr/bin/env python3

'''
coding test invers kineamtiks dengan GUI
'''

from math import sin, cos, acos, atan2, sqrt, radians, degrees

import rospy
from rospy.timer import Rate
from scara.msg import stepperTask, encoder, syncEncoder

data = stepperTask()
data.enable = False
data.stepperTaskList = [0,0,0,0]

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
        global dataEncoder
        dataEncoder = []
        x_pos, y_pos, z_pos = 0.0, 0.0, 0.0

        self.stepper_btn.clicked.connect(self.enable_stepper)
        self.x_target_box.textChanged.connect(self.x_target_changed)
        self.y_target_box.textChanged.connect(self.y_target_changed)
        self.z_target_box.textChanged.connect(self.z_target_changed)
        self.run_ik_btn.clicked.connect(self.run_ik)
        self.sync.clicked.connect(self.sync_handler)

        '''subcribe from sensor data and show it in GUI'''
        self.sub_sensor = rospy.Subscriber("encoder", encoder, self.encoder_callback)

    def x_target_changed(self, value):
        global x_pos
        try:
            x_pos = int(value)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def y_target_changed(self, value):
        global y_pos
        try:
            y_pos = int(value)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def z_target_changed(self, value):
        global z_pos
        try:
            z_pos = int(value)
            print(z_pos)
        except ValueError:
            rospy.loginfo(f"{value} can't be float")

    def run_ik(self):
        x,y,z = x_pos, y_pos, z_pos
        print(x,y,z)
        l1, l2, l3 = 70.2, 152.7, 149.7
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
            a, b, c = tetha1, tetha2, tetha3
            b += a
            c += b
            hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
            hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
            hasil_z = z
            
            data.stepperTaskList[0] = -int(tetha1 * 720 * 2 / 360)
            data.stepperTaskList[1] = int(tetha2 * 730 * 2 / 360)
            data.stepperTaskList[2] = int(tetha3 * 1220 * 2 / 360)
            data.stepperTaskList[3] = -int(z * 7000 * 2 / 360)

            self.tetha1_target_label.setText(str(tetha1))
            self.tetha2_target_label.setText(str(tetha2))
            self.tetha3_target_label.setText(str(tetha3))
            self.tinggi_target_label.setText(str(z))

            self.x_hasil.setText(str(hasil_x))
            self.y_hasil.setText(str(hasil_y))
            self.z_hasil.setText(str(hasil_z))
            #return [tetha1, tetha2, tetha3, z]
            self.publish()

        except ValueError:
            rospy.loginfo('IK Value Out of Reach')
        
    def enable_stepper(self, checked):
        if(checked):
            self.stepper_btn.setText("STEPPER ON")
            data.enable = True
        else:
            self.stepper_btn.setText("STEPPER OFF")
            data.enable = False
        self.publish()
    
    def publish(self):
        pub.publish(data)

    def encoder_callback(self,data):
        global dataEncoder
        tetha1, tetha2, tetha3, tinggi = data.encoderPostList
        tetha1 = -tetha1 * 90 /2400
        tetha2 = -tetha2 * 88.76712328/2400
        tetha3 = -tetha3 * 53.114754098/2400
        tinggi = -tinggi * 9.257142857 / 2400

        dataEncoder = [tetha1, tetha2, tetha3, tinggi]
        
        self.tetha1_real_label.setText(str(tetha1))
        self.tetha2_real_label.setText(str(tetha2))
        self.tetha3_real_label.setText(str(tetha3))
        self.tinggi_real_label.setText(str(tetha3))

    def sync_handler(self):
        tetha1, tetha2, tetha3, tinggi = dataEncoder
        # tetha1 = -tetha1 * 90 /2400
        # tetha2 = -tetha2 * 88.76712328/2400
        # tetha3 = -tetha3 * 53.114754098/2400
        # tinggi = -tinggi * 9.257142857 / 2400
        print(tetha1,tetha2,tetha3)
        
        data.stepperTaskList[0] = -int(tetha1 * 720 * 2 / 360)
        data.stepperTaskList[1] = int(tetha2 * 730 * 2 / 360)
        data.stepperTaskList[2] = int(tetha3 * 1220 * 2 / 360)
        data.stepperTaskList[3] = -int(tinggi * 7000 * 2 / 360)
        self.publish()
        



rospy.init_node('invers_kinematics_gui_rev1')
pub = rospy.Publisher('invers_kinematics_gui_rev1', stepperTask, queue_size=10)
enc_pub = rospy.Publisher('sync_encoder',syncEncoder, queue_size=10)

app = QApplication(sys.argv)
window = MainWindows()

window.show()
app.exec_()