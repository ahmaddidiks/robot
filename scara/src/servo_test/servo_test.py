import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi

import rospy
from std_msgs.msg import UInt16

rospy.init_node('servo')
pub = rospy.Publisher('servo', UInt16, queue_size=10)


class MainWindow(QMainWindow):
    def __init__(self, parent= None):
        super(MainWindow, self).__init__(parent)
        loadUi('/home/didik/robot/src/scara/src/servo_test/servo_test.ui', self)
        self.horizontalSlider.valueChanged.connect(self.servo)

    def servo(self, data):
        pos = 0
        try:
            pos = int(data)
        except:
            pos = 0
            rospy.loginfo("slider value can't be int")
        pub.publish(pos)
        
app = QApplication(sys.argv)
window = MainWindow()

window.show()
app.exec_()
