import rospy

from sensor_msgs.msg import JointState
from scara.msg import stepper
from math import radians, sin, cos
data = stepper()
tetha = []

def get_data(data):
    tetha[1] = data.stepperPostList[0]
    tinggi = data.stepperPostList[1]
    tetha[2] = data.stepperPostList[2]
    tetha[3] = data.stepperPostList[3]
    
    a, b, c = tetha[1], tetha[2], tetha[3]
    b += a
    c += b
    #robot link
    l1, l2, l3 = 70.2, 152.7, 149.7
    #forward kineamtics
    hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
    hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
    hasil_z = tinggi
    print(hasil_x, hasil_y, hasil_z)

while not rospy.is_shutdown():
    rospy.Subscriber(





















































def forward_kinematics():
    tetha = data.stepperPostList
    tetha1 = radians(tetha1)
    tetha2 = radians(tetha2)
    tetha3 = radians(tetha3)
    a, b, c = tetha1, tetha2, tetha3
    b += a
    c += b
    #robot link
    l1, l2, l3 = 70.2, 152.7, 149.7
    #forward kineamtics
    hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
    hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
    hasil_z = tinggi
    return hasil_x, hasil_y, hasil_z
