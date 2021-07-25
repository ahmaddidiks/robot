#!/usr/bin/env python3
from math import sin, cos, acos, atan2, sqrt, radians, degrees
from random import randrange

l1 = 70.20 #mm
l2 = 152.70 #mm
l3 = 149.7 #mm

# x,y,z coordinates in mm
def ik(x,y,z):

    try:
        tetha1 = degrees(atan2(x,y))
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
        z = z #nanti di kali gain / di konvert ke scala / derajat
        return [tetha1, tetha2, tetha3, z]
       
    except ValueError:
        return '[IK_Target] Value Error'
def coba(x,y):
    z=0
    #x,y = 0,369
    tetha1, tetha2, tetha3, z = ik(x,y,z)
    print(tetha1,tetha2,tetha3)
    tetha2 += tetha1 
    tetha3 += tetha2
    hasil_y = l1*cos(radians(tetha1)) + l2*cos(radians(tetha2)) + l3*cos(radians(tetha3))
    hasil_x = l1*sin(radians(tetha1)) + l2*sin(radians(tetha2)) + l3*sin(radians(tetha3))
   
    print(f"max:369.78 -- x={x}, y={y} -- hasil x={hasil_x} hasil y={hasil_y}")


for i in range(100):
    print(f"ITERASI KE {i}")
    coba(randrange(50, 200), randrange(-170, -50))
    

