#!/usr/bin/env python

import numpy as np
import math
from math import sin, cos, pi

G1_endx = np.zeros((60,1))
G1_endy = np.zeros((60,1))
G1_endz = np.zeros((60,1))
G2_endx = np.zeros((60,1))
G2_endy = np.zeros((60,1))
G2_endz = np.zeros((60,1))
G3_endx = np.zeros((60,1))
G3_endy = np.zeros((60,1))
G3_endz = np.zeros((60,1))
G4_endx = np.zeros((60,1))
G4_endy = np.zeros((60,1))
G4_endz = np.zeros((60,1))

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


def S_ForwO(X,Y,Orient,pos2,pos3,pos4):
    Pcom_x = np.vstack((0,X.reshape(60,1)))
    Pcom_y = np.vstack((0,Y.reshape(60,1)))
    Pcom_z = np.ones((60,1))*0.1952
    roll,pitch,yaw = Orient
    Q2_1,Q2_2,Q2_3 = np.hsplit(pos2,3)
    Q3_1,Q3_2,Q3_3 = np.hsplit(pos3,3)
    Q4_1,Q4_2,Q4_3 = np.hsplit(pos4,3)
    for i in range (60):
        G2_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + cos(pitch)*cos(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
        G2_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
        G2_endz[i] = Pcomz[i] - sin(pitch)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(roll)*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i]))
        G3_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + cos(pitch)*cos(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
        G3_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
        G3_endz[i] = Pcomz[i] - sin(pitch)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(roll)*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i]))
        G4_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + cos(pitch)*cos(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
        G4_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
        G4_endz[i] = Pcomz[i] - sin(pitch)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(roll)*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i]))

    k = np.column_stack((G2_endx,G2_endy,G3_endx,G3_endy,G4_endx,G4_endy))
    return k

def S_Forw(Xe,Ye,X,Y,Orient,pos_i,pos_ii,pos_iii,s):
    sequence = s
    Pcom_x = np.vstack((Xe,X.reshape(60,1)))
    Pcom_y = np.vstack((Ye,Y.reshape(60,1)))
    Pcom_z = np.ones((60,1))*0.1952
    roll,pitch,yaw = Orient

    if (sequence == 0):
        Q2_1,Q2_2,Q2_3 = np.hsplit(pos_i,3)
        Q3_1,Q3_2,Q3_3 = np.hsplit(pos_ii,3)
        Q4_1,Q4_2,Q4_3 = np.hsplit(pos_iii,3)
        for i in range (60):
            G2_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + cos(pitch)*cos(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endz[i] = Pcomz[i] - sin(pitch)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(roll)*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i]))
            G3_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + cos(pitch)*cos(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endz[i] = Pcomz[i] - sin(pitch)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(roll)*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i]))
            G4_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + cos(pitch)*cos(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endz[i] = Pcomz[i] - sin(pitch)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(roll)*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i]))

        k = np.column_stack((G2_endx,G2_endy,G3_endx,G3_endy,G4_endx,G4_endy))

    elif (sequence == 1):
        Q1_1,Q1_2,Q1_3 = np.hsplit(pos_i,3)
        Q2_1,Q2_2,Q2_3 = np.hsplit(pos_ii,3)
        Q4_1,Q4_2,Q4_3 = np.hsplit(pos_iii,3)
        for i in range (60):
            G1_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + cos(pitch)*cos(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endz[i] = Pcomz[i] - sin(pitch)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(roll)*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i]))
            G2_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + cos(pitch)*cos(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endz[i] = Pcomz[i] - sin(pitch)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(roll)*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i]))
            G4_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + cos(pitch)*cos(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endz[i] = Pcomz[i] - sin(pitch)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(roll)*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i]))

        k = np.column_stack((G1_endx,G1_endy,G2_endx,G2_endy,G4_endx,G4_endy))

    elif (sequence == 2):
        Q1_1,Q1_2,Q1_3 = np.hsplit(pos_i,3)
        Q3_1,Q3_2,Q3_3 = np.hsplit(pos_ii,3)
        Q4_1,Q4_2,Q4_3 = np.hsplit(pos_iii,3)
        for i in range (60):
            G1_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + cos(pitch)*cos(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endz[i] = Pcomz[i] - sin(pitch)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(roll)*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i]))
            G3_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + cos(pitch)*cos(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endz[i] = Pcomz[i] - sin(pitch)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(roll)*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i]))
            G4_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + cos(pitch)*cos(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(yaw)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i])))
            G4_endz[i] = Pcomz[i] - sin(pitch)*(X4 + cos(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) + cos(pitch)*sin(roll)*(Y4 + sin(Q4_1[i] + t_z4)*(L1 + L2*cos(Q4_2[i]) + L3*cos(Q4_2[i]+Q4_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q4_2[i] + Q4_3[i]) + L2*sin(Q4_2[i]))

        k = np.column_stack((G1_endx,G1_endy,G3_endx,G3_endy,G4_endx,G4_endy))

    else:
        Q1_1,Q1_2,Q1_3 = np.hsplit(pos_i,3)
        Q2_1,Q2_2,Q2_3 = np.hsplit(pos_ii,3)
        Q3_1,Q3_2,Q3_3 = np.hsplit(pos_iii,3)
        for i in range (60):
            G1_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + cos(pitch)*cos(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(yaw)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i])))
            G1_endz[i] = Pcomz[i] - sin(pitch)*(X1 + cos(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) + cos(pitch)*sin(roll)*(Y1 + sin(Q1_1[i] + t_z1)*(L1 + L2*cos(Q1_2[i]) + L3*cos(Q1_2[i]+Q1_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q1_2[i] + Q1_3[i]) + L2*sin(Q1_2[i]))
            G2_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + cos(pitch)*cos(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(yaw)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i])))
            G2_endz[i] = Pcomz[i] - sin(pitch)*(X2 + cos(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) + cos(pitch)*sin(roll)*(Y2 + sin(Q2_1[i] + t_z2)*(L1 + L2*cos(Q2_2[i]) + L3*cos(Q2_2[i]+Q2_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q2_2[i] + Q2_3[i]) + L2*sin(Q2_2[i]))
            G3_endx[i] = Pcomx[i] - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + cos(pitch)*cos(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endy[i] = Pcomy[i] + (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i])) + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(yaw)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i])))
            G3_endz[i] = Pcomz[i] - sin(pitch)*(X3 + cos(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) + cos(pitch)*sin(roll)*(Y3 + sin(Q3_1[i] + t_z3)*(L1 + L2*cos(Q3_2[i]) + L3*cos(Q3_2[i]+Q3_3[i]))) - cos(pitch)*cos(roll)*(L3*sin(Q3_2[i] + Q3_3[i]) + L2*sin(Q3_2[i]))

        k = np.column_stack((G1_endx,G1_endy,G2_endx,G2_endy,G3_endx,G3_endy))

    return k
