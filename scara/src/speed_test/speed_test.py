#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

quaternion = Quaternion() #[x,y,z,w]
quaternion.w = 1 #static speed
stepper = Bool()

rospy.init_node('robot_position')
pub = rospy.Publisher('position', Quaternion, queue_size=10)
        
class MainWindow(QMainWindow):
    def __init__(self, parent= None):
        super(MainWindow, self).__init__(parent)
        loadUi('/home/didik/robot/src/scara/src/speed_test/gui.ui', self)
        self.x_slider.valueChanged.connect(self.x_slide)
        self.y_slider.valueChanged.connect(self.y_slide)
        self.z_slider.valueChanged.connect(self.z_slide)

    def x_slide(self, data):
        quaternion.x = -int(data)
        pub.publish(quaternion)

    def y_slide(self, data):
        quaternion.y = int(data)
        pub.publish(quaternion)

    def z_slide(self, data):
        quaternion.z = int(data)
        pub.publish(quaternion)

app = QApplication(sys.argv)
window = MainWindow()

window.show()
app.exec_()