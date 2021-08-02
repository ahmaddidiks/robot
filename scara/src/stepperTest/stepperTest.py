#!/usr/bin/env python3

'''
coding test stepper test
'''

import rospy
from rospy.timer import Rate
from scara.msg import stepper

import sys
from PyQt5.QtWidgets import  QMainWindow, QApplication, QDialog
from PyQt5.uic import loadUi

#global ROS var (scara msgs)
data = stepper()
data.enableStepper = False
data.stepperPostList = [0,0,0,0]

#inisasi node
rospy.init_node("stepper_test_gui")
rate = Rate(100)
pub = rospy.Publisher("stepper_test_gui", stepper, queue_size=10)

class MainUI(QMainWindow):
    def __init__(self):
        super(MainUI, self).__init__()
        loadUi("/home/didik/robot/src/scara/src/stepperTest/stepperTest.ui", self)
        self.setWindowTitle("Stepper Test GUI")
        self.pushButton.clicked.connect(self.buttonHandler)
        self.slider0.valueChanged.connect(self.slider0Changed)
        self.slider1.valueChanged.connect(self.slider1Changed)
        self.slider2.valueChanged.connect(self.slider2Changed)
        self.slider3.valueChanged.connect(self.slider3Changed)

    def buttonHandler(self, checked):
        if(checked):
            self.pushButton.setText("ON, Click to turn OFF")
            data.enableStepper = True
            self.publish()

        else:
            self.pushButton.setText("OFF, Click to turn ON")
            data.enableStepper = False
            self.publish()
    
    def slider0Changed(self, text):
        try:
            data.stepperPostList[0] = float(text)
            self.publish()
        except:
            rospy.loginfo("slider 0 ERROR")
    def slider1Changed(self, text):
        try:
            data.stepperPostList[1] = float(text)
            self.publish()
        except:
            rospy.loginfo("slider 1 ERROR")
    def slider2Changed(self, text):
        try:
            data.stepperPostList[2] = float(text)
            self.publish()
        except:
            rospy.loginfo("slider 2 ERROR")
    def slider3Changed(self, text):
        try:
            data.stepperPostList[3] = -float(text)
            self.publish()
        except:
            rospy.loginfo("slider 3 ERROR")

    def publish(self):
        pub.publish(data)
        rospy.loginfo(data)

app = QApplication(sys.argv)
window = MainUI()

window.show()
app.exec_()

