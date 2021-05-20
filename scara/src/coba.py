import math
import numpy as np

#length in mm
l0 = 20
l1 = 100
l2 = 100
l3 = 0

def cal(x,y,z):
    A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
    B = math.sqrt(1-(A**2))
    tetha3 = math.degrees(math.acos(A))
           
    buff1 = l2*math.cos(math.radians(tetha3)) + l1
    buff2 = math.sqrt(x**2 + y**2)
    buff3 = l2*math.sin(math.radians(tetha3))

    C = math.degrees(math.atan2(buff2,z))
    D = math.degrees(math.atan2(buff3,buff1))
    tetha2 = C - D
    tetha5 = math.degrees(math.atan2(x,y))
    tetha1 = tetha5
    tetha4 = tetha3 + tetha2 - 90
    return [tetha1, tetha2, tetha3]

print(cal(2,3,4))