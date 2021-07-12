#!/usr/bin/env python3

import rospy
from scara.msg import link
data = link()

import sys
from PyQt5.QtWidgets import  QMainWindow, QApplication, QDialog
from PyQt5.uic import loadUi


#inisiasi node
rospy.init_node("scara_urdf_gui_test")
pub = rospy.Publisher("scara_urdf_gui", link, queue_size=10)

class MainUI(QMainWindow):
    def __init__(self):
        super(MainUI, self).__init__()
        loadUi('/home/didik/robot/src/scara/src/scara_urdf_gui_test/ui.ui',self)
        self.setWindowTitle("SCARA URDF GUI TEST")
        self.link1.valueChanged.connect(self.link1_handler)
        self.link2.valueChanged.connect(self.link2_handler)
        self.link3.valueChanged.connect(self.link3_handler)
        self.z.valueChanged.connect(self.z_handler)

    def link1_handler(self, value):
        data.link1 = int(value)
        self.publish()

    def link2_handler(self, value):
        data.link2 = int(value)
        self.publish()

    def link3_handler(self, value):
        data.link3 = int(value)
        self.publish()

    def z_handler(self, value):
        data.z = int(value)
        self.publish()   
        
    def publish(self):
        pub.publish(data)
        rospy.loginfo(data)

app = QApplication(sys.argv)
window = MainUI()

window.show()
app.exec_()