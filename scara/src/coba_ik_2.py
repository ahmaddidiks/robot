#!/usr/bin/env python3
from math import sin, cos, acos, atan2, sqrt, radians, degrees, atan
'''
OP3 ROBOT ID MAPING BASED
tehta 1 ==> ID 9,10
tehta 2 ==> ID 11,12
tehta 3 ==> ID 13,14
tehta 4 ==> ID 15,16
tehta 5 ==> ID 17,18
tehta 6 ==> ID 7,8
'''
l1 = 152.26 #mm
l2 = 149.23 #mm
l3 = 30.00 #mm


def ik2(x,y,z):
    try:
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = sqrt(1-(A**2))
        tetha3 = degrees(atan2(B,A))
        
        buff1 = (l2*cos(radians(tetha3)))+l1
        buff2 = sqrt(x**2 + y**2)
        buff3 = l2*sin(tetha3)

        C = (z*buff1) - (buff2*buff3)
        D = (buff1*buff2) + (y*buff3)

        tetha2 = degrees(atan2(C,D))
        tetha1 = degrees(atan2(x,y))
        #return x,y,z
        return [tetha1, tetha2, tetha3]
    except:
        return 'tidak ada hasil'


# x,y,z coordinates in mm
def ik(koordinatX, koordinatY,  koordinatZ):
    x = koordinatX ## ==> y in robotics ==> nilai selalu nol    x
    y = koordinatZ ## ==> x in robotics ==> menjadi y           y
    z = koordinatY ## ==> menjadi x                             z
    '''
    rotate y(x) axis -90deg ==> doesn't need x(y) coordinates (always 0) ==> theta 1
    '''
    try:
        tetha1 = degrees(atan(y/z))
        z = 0
        y = y - l3
        #
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = sqrt(1-(A**2))
        tetha3 = degrees(acos(A))
        
        buff1 = l2*cos(radians(tetha3)) + l1
        buff2 = sqrt(x**2 + z**2)
        buff3 = l2*sin(radians(tetha3))

        C = degrees(atan2(buff2,z))
        D = degrees(atan2(buff3,buff1))
        tetha2 = C - D
        
        return [tetha1, tetha2, tetha3]
        #return [tetha1+180, 90+tetha2, tetha3+180, tetha4+180, tetha5+180] #[tetha2 + 90, tetha3+180

    except ValueError:
        return '[IK_Target] Value Error'
def coba1():
    z=0
    x,y = -200,251
    c, a, b = ik(z,x,y)
    print(c,a,b)
    b +=a
    hasil_y, hasil_x = l1*cos(radians(a)) + l2*cos(radians(b)) , l1*sin(radians(a)) + l2*sin(radians(b))
    print(f"{l1+l2} -- x=0, y={y-l3} --jarak(abs) hasil x={hasil_x} hasil y={hasil_y}")

def coba2():
    z=0
    x,y = 0,250
    a, b, c = ik2(x,y,z)
    print(a,b)
    b +=a
    hasil_y, hasil_x = l1*cos(radians(a)) + l2*cos(radians(b)) , l1*sin(radians(a)) + l2*sin(radians(b))
    print(f"{l1+l2} -- x={x}, y={y} --jarak(abs) hasil x={hasil_x} hasil y={hasil_y}")


coba1()
