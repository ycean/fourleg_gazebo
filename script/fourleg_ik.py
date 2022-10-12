#!/usr/bin/env python
#Inverse Kinematic with Pieper's solution


import numpy as np
from math import pi, cos, sin, atan2, hypot, acos, atan, sqrt, asin

L1 = 0.114
L2 = 0.1389
L3 = 0.2484
t_z1 = 2.445
t_z2 = 0.6967
t_z3 = -0.6967
t_z4 = -2.445
X1 = -0.1536
Y1 = 0.1285
X2 = 0.1536
Y2 = 0.1285
X3 = 0.1536
Y3 = -0.1285
X4 = -0.1536
Y4 = -0.1285


def IK_Leg1(Pcomx,Pcomy,Pcomz,Orient,P1_x,P1_y,P1_z):
    roll,pitch,yaw = Orient
    #The Ground reference for the position of First point of leg can be calculate as: P1_GB
    GO1_x = Pcomx - Y1*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + X1*cos(pitch)*cos(yaw)
    GO1_y = Pcomy + Y1*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + X1*cos(pitch)*sin(yaw)
    GO1_z = Pcomz - X1*sin(pitch) + Y1*cos(pitch)*sin(roll)

    Q1_1_pre = atan2(P1_y-GO1_y,P1_x-GO1_x)
    Q1_1 = Q1_1_pre - yaw + 0.6966
    if abs(Q1_1)>pi/2:
        if Q1_1 > 0:
            Q1_1 = Q1_1 - pi
        else:
            Q1_1 = Q1_1 + pi
    else:
        Q1_1 = Q1_1

    LD = sqrt((P1_y-GO1_y)**2+(P1_x-GO1_x)**2)
    L_sq = (LD-L1)**2+(Pcomz-P1_z)**2
    Q1_3 = pi - acos((L2**2+L3**2-L_sq)/(2*L2*L3))
    L = sqrt(L_sq)
    Q1_2 = -(asin((L3/L)*sin(pi-Q1_3)) - atan2(Pcomz-P1_z,LD-L1))

    pos1 = np.array([Q1_1,Q1_2,Q1_3])
    return pos1

def IK_Leg2(Pcomx,Pcomy,Pcomz,Orient,P2_x,P2_y,P2_z):
    roll,pitch,yaw = Orient
    #The Ground reference for the position of First point of leg can be calculate as: P2_GB
    GO2_x = Pcomx - Y2*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + X2*cos(pitch)*cos(yaw)
    GO2_y = Pcomy + Y2*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + X2*cos(pitch)*sin(yaw)
    GO2_z = Pcomz - X2*sin(pitch) + Y2*cos(pitch)*sin(roll)

    Q2_1_pre = atan2(P2_y-GO2_y,P2_x-GO2_x)
    Q2_1 = Q2_1_pre - yaw - 0.6966
    if abs(Q2_1)>pi/2:
        if Q2_1 > 0:
            Q2_1 = Q2_1 - pi
        else:
            Q2_1 = Q2_1 + pi
    else:
        Q2_1 = Q2_1

    LD = sqrt((P2_y-GO2_y)**2+(P2_x-GO2_x)**2)
    L_sq = (LD-L1)**2+(Pcomz-P2_z)**2
    Q2_3 = pi - acos((L2**2+L3**2-L_sq)/(2*L2*L3))

    L = sqrt(L_sq)
    Q2_2 = -(asin((L3/L)*sin(pi-Q2_3)) - atan2(Pcomz-P2_z,LD-L1))

    pos2 = np.array([Q2_1,Q2_2,Q2_3])
    return pos2

def IK_Leg3(Pcomx,Pcomy,Pcomz,Orient,P3_x,P3_y,P3_z):
    roll,pitch,yaw = Orient
    #The Ground reference for the position of First point of leg can be calculate as: P3_GB
    GO3_x = Pcomx - Y3*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + X3*cos(pitch)*cos(yaw)
    GO3_y = Pcomy + Y3*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + X3*cos(pitch)*sin(yaw)
    GO3_z = Pcomz - X3*sin(pitch) + Y3*cos(pitch)*sin(roll)

    Q3_1_pre = atan2(P3_y-GO3_y,P3_x-GO3_x)
    Q3_1 = Q3_1_pre - yaw + 0.6966
    if abs(Q3_1) > pi/2:
        if Q3_1 > 0:
            Q3_1 = Q3_1 - pi
        else:
            Q3_1 = Q3_1 + pi
    else:
        Q3_1 = Q3_1

    LD = sqrt((P3_y-GO3_y)**2+(P3_x-GO3_x)**2)
    L_sq = (LD-L1)**2+(Pcomz-P3_z)**2
    Q3_3 = pi - acos((L2**2+L3**2-L_sq)/(2*L2*L3))

    L = sqrt(L_sq)
    Q3_2 = -(asin((L3/L)*sin(pi-Q3_3)) - atan2(Pcomz-P3_z,LD-L1))

    pos3 = np.array([Q3_1,Q3_2,Q3_3])
    return pos3

def IK_Leg4(Pcomx,Pcomy,Pcomz,Orient,P4_x,P4_y,P4_z):
    roll,pitch,yaw = Orient
    #The Ground reference for the position of First point of leg can be calculate as: P4_GB
    GO4_x = Pcomx - Y4*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + X4*cos(pitch)*cos(yaw)
    GO4_y = Pcomy + Y4*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + X4*cos(pitch)*sin(yaw)
    GO4_z = Pcomz - X4*sin(pitch) + Y4*cos(pitch)*sin(roll)

    Q4_1_pre = atan2(P4_y-GO4_y,P4_x-GO4_x)
    Q4_1 = Q4_1_pre - yaw - 0.6966
    if abs(Q4_1) > pi/2:
        if Q4_1 > 0:
            Q4_1 = Q4_1 - pi
        else:
            Q4_1 = Q4_1 + pi
    else:
        Q4_1 = Q4_1

    LD = sqrt((P4_y-GO4_y)**2+(P4_x-GO4_x)**2)
    L_sq = (LD-L1)**2+(Pcomz-P4_z)**2
    Q4_3 = pi - acos((L2**2+L3**2-L_sq)/(2*L2*L3))

    L = sqrt(L_sq)
    Q4_2 = -(asin((L3/L)*sin(pi-Q4_3)) - atan2(Pcomz-P4_z,LD-L1))

    pos4 = np.array([Q4_1,Q4_2,Q4_3])
    return pos4
