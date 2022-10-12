#!/usr/bin/env python
#defining foot step
# PLAN FOR WALKING TO RIGHT SIDE
#USED IN FV3

from math import cos, sin, pi, hypot, atan2
import numpy as np

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

def FS_define_R0(Pcomx,Pcomy,Pcomz,Orient,fs,loop = None):

    roll,pitch,yaw = Orient


    f_sq = fs % 4
    #Walking Sequence : 2 - 4 - 3 - 1
    if (f_sq == 0): #Lift leg 2 turn leg 1
        if (loop == 0): #Initial step
            Q1_1 = 0
            Q1_2 = -pi/8
            Q1_3 = 5*pi/8
            Q2_1 = 0
            Q2_2 = -pi/8
            Q2_3 = 5*pi/8
            Q3_1 = 0
            Q3_2 = -pi/8
            Q3_3 = 5*pi/8
            Q4_1 = 0
            Q4_2 = -pi/8
            Q4_3 = 5*pi/8
        else:
            Q1_1 = -pi/4    # leg 1 turn in -ve
            Q1_2 = -pi/8
            Q1_3 = 5*pi/8
            Q2_1 = 0
            Q2_2 = -pi/8
            Q2_3 = 5*pi/8
            Q3_1 = 0
            Q3_2 = -pi/8
            Q3_3 = 5*pi/8
            Q4_1 = 0
            Q4_2 = -pi/8
            Q4_3 = 5*pi/8
    elif(f_sq == 1): #Lift leg 4 turn leg 2
        Q1_1 = 0
        Q1_2 = -pi/8
        Q1_3 = 5*pi/8
        Q2_1 = -pi/8    #SMALLER ANGLE FOR FRONT LEG
        Q2_2 = -pi/8    # leg 2 turn in -ve
        Q2_3 = 5*pi/8
        Q3_1 = 0
        Q3_2 = -pi/8
        Q3_3 = 5*pi/8
        Q4_1 = 0
        Q4_2 = -pi/8
        Q4_3 = 5*pi/8
    elif(f_sq == 2): #Lift leg 3 turn leg 4
        Q1_1 = 0
        Q1_2 = -pi/8
        Q1_3 = 5*pi/8
        Q2_1 = 0
        Q2_2 = -pi/8
        Q2_3 = 5*pi/8
        Q3_1 = 0
        Q3_2 = -pi/8
        Q3_3 = 5*pi/8
        Q4_1 = pi/4     # leg 4 turn in +ve
        Q4_2 = -pi/8
        Q4_3 = 5*pi/8
    elif(f_sq == 3):#Lift leg 1 turn leg 3
        Q1_1 = 0
        Q1_2 = -pi/8
        Q1_3 = 5*pi/8
        Q2_1 = 0
        Q2_2 = -pi/8
        Q2_3 = 5*pi/8
        Q3_1 = pi/8    #SMALLER ANGLE FOR FRONT LEG     # leg 3 turn in +ve
        Q3_2 = -pi/8
        Q3_3 = 5*pi/8
        Q4_1 = 0
        Q4_2 = -pi/8
        Q4_3 = 5*pi/8

    #Leg1:The Ground reference for the position of end point of leg can be calculate as: P1_G3
    P1_x = Pcomx - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y1 + sin(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q1_2 + Q1_3) + L2*sin(Q1_2)) + cos(pitch)*cos(yaw)*(X1 + cos(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3)))
    P1_y = Pcomy + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q1_2 + Q1_3) + L2*sin(Q1_2)) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y1 + sin(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3))) + cos(pitch)*sin(yaw)*(X1 + cos(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3)))
    P1_z = Pcomz - sin(pitch)*(X1 + cos(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3))) + cos(pitch)*sin(roll)*(Y1 + sin(Q1_1 + t_z1)*(L1 + L2*cos(Q1_2) + L3*cos(Q1_2+Q1_3))) - cos(pitch)*cos(roll)*(L3*sin(Q1_2 + Q1_3) + L2*sin(Q1_2))
    #Leg2:The Ground reference for the position of end point of leg can be calculate as: P2_G3
    P2_x = Pcomx - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y2 + sin(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q2_2 + Q2_3) + L2*sin(Q2_2)) + cos(pitch)*cos(yaw)*(X2 + cos(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3)))
    P2_y = Pcomy + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q2_2 + Q2_3) + L2*sin(Q2_2)) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y2 + sin(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3))) + cos(pitch)*sin(yaw)*(X2 + cos(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3)))
    P2_z = Pcomz - sin(pitch)*(X2 + cos(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3))) + cos(pitch)*sin(roll)*(Y2 + sin(Q2_1 + t_z2)*(L1 + L2*cos(Q2_2) + L3*cos(Q2_2+Q2_3))) - cos(pitch)*cos(roll)*(L3*sin(Q2_2 + Q2_3) + L2*sin(Q2_2))
    #Leg3:The Ground reference for the position of end point of leg can be calculate as: P3_G3
    P3_x = Pcomx - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y3 + sin(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q3_2 + Q3_3) + L2*sin(Q3_2)) + cos(pitch)*cos(yaw)*(X3 + cos(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3)))
    P3_y = Pcomy + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q3_2 + Q3_3) + L2*sin(Q3_2)) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y3 + sin(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3))) + cos(pitch)*sin(yaw)*(X3 + cos(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3)))
    P3_z = Pcomz - sin(pitch)*(X3 + cos(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3))) + cos(pitch)*sin(roll)*(Y3 + sin(Q3_1 + t_z3)*(L1 + L2*cos(Q3_2) + L3*cos(Q3_2+Q3_3))) - cos(pitch)*cos(roll)*(L3*sin(Q3_2 + Q3_3) + L2*sin(Q3_2))
    #Leg4:The Ground reference for the position of end point of leg can be calculate as: P4_G3
    P4_x = Pcomx - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y4 + sin(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q4_2 + Q4_3) + L2*sin(Q4_2)) + cos(pitch)*cos(yaw)*(X4 + cos(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3)))
    P4_y = Pcomy + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q4_2 + Q4_3) + L2*sin(Q4_2)) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y4 + sin(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3))) + cos(pitch)*sin(yaw)*(X4 + cos(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3)))
    P4_z = Pcomz - sin(pitch)*(X4 + cos(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3))) + cos(pitch)*sin(roll)*(Y4 + sin(Q4_1 + t_z4)*(L1 + L2*cos(Q4_2) + L3*cos(Q4_2+Q4_3))) - cos(pitch)*cos(roll)*(L3*sin(Q4_2 + Q4_3) + L2*sin(Q4_2))


    P1 = [P1_x,P1_y]

    P2 = [P2_x,P2_y]

    P3 = [P3_x,P3_y]

    P4 = [P4_x,P4_y]

    if (f_sq == 0):#Lift leg 2 turn leg 1
        print("P4,P3,P1,P4")
        fs_X = [round(P4[0],4),round(P3[0],4),round(P1[0],4),round(P4[0],4)]
        fs_Y = [round(P4[1],4),round(P3[1],4),round(P1[1],4),round(P4[1],4)]
    elif(f_sq == 1):#Lift leg 4 turn leg 2
        print("P3,P1,P2,P3")
        fs_X = [round(P3[0],4),round(P1[0],4),round(P2[0],4),round(P3[0],4)]
        fs_Y = [round(P3[1],4),round(P1[1],4),round(P2[1],4),round(P3[1],4)]
    elif(f_sq == 2):#Lift leg 3 turn leg 4
        print("P2,P1,P4,P2")
        fs_X = [round(P2[0],4),round(P1[0],4),round(P4[0],4),round(P2[0],4)]
        fs_Y = [round(P2[1],4),round(P1[1],4),round(P4[1],4),round(P2[1],4)]
    elif(f_sq == 3):#Lift leg 1 turn leg 3
        print("P3,P4,P2,P3")
        fs_X = [round(P3[0],4),round(P4[0],4),round(P2[0],4),round(P3[0],4)]
        fs_Y = [round(P3[1],4),round(P4[1],4),round(P2[1],4),round(P3[1],4)]


    new_fs = np.array([fs_X,fs_Y]).reshape(2,4)
    #old_fs = np.append(fs[0][4:8],fs[1][4:8],0).reshape(2,4)
    #fs_new = np.append(old_fs,new_fs,1)
    return new_fs
