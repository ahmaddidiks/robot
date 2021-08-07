import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from math import radians, sin, cos
data = JointState()
tetha = [0,0,0,0]
last_x, last_y, last_z = 0,0,0

traj = Path()
traj.poses = 

rospy.init_node("data_software")
rate = rospy.Rate(60)


def get_data(data):
    global last_z, last_y, last_x, tetha
    tetha[1] = JointState().position[0]
    tinggi = JointState().position[1]
    tetha[2] = JointState().position[2]
    tetha[3] = JointState().position[3]
    
    a, b, c = tetha[1], tetha[2], tetha[3]
    b += a
    c += b
    #robot link
    l1, l2, l3 = 70.2, 152.7, 149.7
    #forward kineamtics
    hasil_y = l1*cos(radians(a)) + l2*cos(radians(b)) + l3*cos(radians(c))
    hasil_x = l1*sin(radians(a)) + l2*sin(radians(b)) + l3*sin(radians(c))
    hasil_z = tinggi

    if(hasil_x != last_x or hasil_y != last_y or hasil_z != last_z):
        last_x = hasil_x
        last_y = hasil_y
        last_z = hasil_z
        print(hasil_x, hasil_y, hasil_z)

while not rospy.is_shutdown():
    rospy.Subscriber('joint_states', JointState, get_data)
    rate.sleep()