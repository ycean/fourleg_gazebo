#!/usr/bin/env python
# step sequence is depending of the direction of the walking plan
# all joint position are published at the same time interval
# remain the stance leg position in the continuous step [update the stance leg's feet position from the calculation of the third link of the respective leg's position from gazebo]
# This script including ros-gazebo, com trajectory optimization, inverse kinematic
# the turing angle for the front legs smaller than the hind legs in footstep predefine for minimizing the robot yaw angle during the walking motion.
#THIS IS SCRIPT IS THE MODIFIED VERSION OF JP_COM_QP_OMNI_DIR_FV3.py and JP_COM_OMNI_DIR_FV2_v2.py

import rospy
import time
import numpy as np
from numpy import arcsin, sign
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import LinkStates, ContactState
from math import sin, cos, pi, pow, atan2, sqrt, ceil, atan, hypot
import tf.transformations
from tf.transformations import*
import sys
import gurobipy as gurobipy
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import matplotlib.animation as ani
import FS_define0
import FS_define_R0
import FS_define_B0
import FS_define_L0
import fourleg_ik
import Gurobi_op
import S_forw
from FS_define0 import*
from FS_define_R0 import*
from FS_define_B0 import*
from FS_define_L0 import*
from fourleg_ik import*
from Gurobi_op import*
from S_forw import*

distance_tolerance = 0.10


class JointPub(object):
    def __init__(self):

        rospy.init_node("joint_publisher_node")     ##init the node

        self.publishers_array1 = []
        self.publishers_array2 = []
        self.publishers_array3 = []
        self.publishers_array4 = []
        self._leg1_joint1_pub = rospy.Publisher("/fourleg/leg1_j1_position_controller/command", Float64, queue_size=1)
        self._leg1_joint2_pub = rospy.Publisher("/fourleg/leg1_j2_position_controller/command", Float64, queue_size=1)
        self._leg1_joint3_pub = rospy.Publisher("/fourleg/leg1_j3_position_controller/command", Float64, queue_size=1)
        self._leg2_joint1_pub = rospy.Publisher("/fourleg/leg2_j1_position_controller/command", Float64, queue_size=1)
        self._leg2_joint2_pub = rospy.Publisher("/fourleg/leg2_j2_position_controller/command", Float64, queue_size=1)
        self._leg2_joint3_pub = rospy.Publisher("/fourleg/leg2_j3_position_controller/command", Float64, queue_size=1)
        self._leg3_joint1_pub = rospy.Publisher("/fourleg/leg3_j1_position_controller/command", Float64, queue_size=1)
        self._leg3_joint2_pub = rospy.Publisher("/fourleg/leg3_j2_position_controller/command", Float64, queue_size=1)
        self._leg3_joint3_pub = rospy.Publisher("/fourleg/leg3_j3_position_controller/command", Float64, queue_size=1)
        self._leg4_joint1_pub = rospy.Publisher("/fourleg/leg4_j1_position_controller/command", Float64, queue_size=1)
        self._leg4_joint2_pub = rospy.Publisher("/fourleg/leg4_j2_position_controller/command", Float64, queue_size=1)
        self._leg4_joint3_pub = rospy.Publisher("/fourleg/leg4_j3_position_controller/command", Float64, queue_size=1)

        self.publishers_array1.append(self._leg1_joint1_pub)
        self.publishers_array1.append(self._leg1_joint2_pub)
        self.publishers_array1.append(self._leg1_joint3_pub)
        self.publishers_array2.append(self._leg2_joint1_pub)
        self.publishers_array2.append(self._leg2_joint2_pub)
        self.publishers_array2.append(self._leg2_joint3_pub)
        self.publishers_array3.append(self._leg3_joint1_pub)
        self.publishers_array3.append(self._leg3_joint2_pub)
        self.publishers_array3.append(self._leg3_joint3_pub)
        self.publishers_array4.append(self._leg4_joint1_pub)
        self.publishers_array4.append(self._leg4_joint2_pub)
        self.publishers_array4.append(self._leg4_joint3_pub)

        self.pose_subscriber = rospy.Subscriber('/gazebo/link_states',LinkStates, self.update_pose)
        self.leg1_pose_subscriber = rospy.Subscriber('/gazebo/link_states',LinkStates, self.update_leg1_pose)
        self.leg2_pose_subscriber = rospy.Subscriber('/gazebo/link_states',LinkStates, self.update_leg2_pose)
        self.leg3_pose_subscriber = rospy.Subscriber('/gazebo/link_states',LinkStates, self.update_leg3_pose)
        self.leg4_pose_subscriber = rospy.Subscriber('/gazebo/link_states',LinkStates, self.update_leg4_pose)

        self.imu_subscriber = rospy.Subscriber('/fourleg/imu/data',Imu, self.update_orientation)

        self.pose = LinkStates()
        self.orientation = Imu()

        self.leg1_pose = LinkStates()
        self.leg2_pose = LinkStates()
        self.leg3_pose = LinkStates()
        self.leg4_pose = LinkStates()

        self.b = 0
        self.c = 0
        self.d = 0
        self.st = 0
        self.time = 0
        self.L1J = np.array([])
        self.L2J = np.array([])
        self.L3J = np.array([])
        self.L4J = np.array([])
        self.Re_x = np.array([])
        self.Re_y = np.array([])
        self.Ac_x = np.array([])
        self.Ac_y = np.array([])
        self.curr_n_goal = np.array([])
        self.Roll = np.array([])
        self.Pitch = np.array([])
        self.Yaw = np.array([])
        self.t = np.array([])
        self.start_time = 0
        self.end_time = 0

    def plot_trajectory(self, x, y, mkr='b-'):
        plt.cla()
        plt.plot(x, y)
        min_x = min(x) - 0.5
        max_x = max(x) + 0.5
        min_y = min(y) - 0.5
        max_y = max(y) + 0.5
        plt.axis([min_x, max_x, min_y, max_y])
        plt.grid()

    def update_pose(self, data):
        self.pose = data.pose[1].position
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.z = round(self.pose.z, 4)
        self.last_recieved_stamp = rospy.Time.now()

    def update_orientation(self, data):
        self.orientation = data.orientation
        self.orientation.x = round(self.orientation.x, 4)
        self.orientation.y = round(self.orientation.y, 4)
        self.orientation.z = round(self.orientation.z, 4)
        self.orientation.w = round(self.orientation.w, 4)
        #self.twist = data.twist[1]
        self.last_recieved_stamp = rospy.Time.now()

    def update_euler_angle(self,data,a):
        x = round(self.orientation.x, 4)
        y = round(self.orientation.y, 4)
        z = round(self.orientation.z, 4)
        w = round(self.orientation.w, 4)

        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])

        if a == 0:
            angle = roll
        elif a == 1:
            angle = pitch
        elif a == 2:
            angle = yaw
        else:
            angle = euler_from_quaternion([x,y,z,w])

        return angle

    def update_leg1_pose(self, data):
        self.leg1_pose = data.pose[4].position #leg 1 link 3 is [4]
        self.leg1x = round(self.leg1_pose.x, 4)
        self.leg1y = round(self.leg1_pose.y, 4)
        self.leg1z = round(self.leg1_pose.z, 4)
        self.last_recieved_stamp = rospy.Time.now()

        self.leg1_orientation = data.pose[4].orientation
        x = round(self.leg1_orientation.x, 4)
        y = round(self.leg1_orientation.y, 4)
        z = round(self.leg1_orientation.z, 4)
        w = round(self.leg1_orientation.w, 4)
        self.last_recieved_stamp = rospy.Time.now()

        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])

        del_1 = sqrt(abs(0.2484**2 - (self.leg1z)**2))
        self.feet_1_x = self.leg1x + cos(yaw)*del_1
        self.feet_1_y = self.leg1y + sin(yaw)*del_1

    def update_leg2_pose(self, data):
        self.leg2_pose = data.pose[7].position #leg 2 link 3 is [7]
        self.leg2x = round(self.leg2_pose.x, 4)
        self.leg2y = round(self.leg2_pose.y, 4)
        self.leg2z = round(self.leg2_pose.z, 4)
        self.last_recieved_stamp = rospy.Time.now()

        self.leg2_orientation = data.pose[7].orientation
        x = round(self.leg2_orientation.x, 4)
        y = round(self.leg2_orientation.y, 4)
        z = round(self.leg2_orientation.z, 4)
        w = round(self.leg2_orientation.w, 4)
        self.last_recieved_stamp = rospy.Time.now()

        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])

        del_2 = sqrt(abs(0.2484**2 - (self.leg2z)**2))
        self.feet_2_x = self.leg2x + cos(yaw)*del_2
        self.feet_2_y = self.leg2y + sin(yaw)*del_2

    def update_leg3_pose(self, data):
        self.leg3_pose = data.pose[10].position #leg 3 link 3 is [10]
        self.leg3x = round(self.leg3_pose.x, 4)
        self.leg3y = round(self.leg3_pose.y, 4)
        self.leg3z = round(self.leg3_pose.z, 4)
        self.last_recieved_stamp = rospy.Time.now()

        self.leg3_orientation = data.pose[10].orientation
        x = round(self.leg3_orientation.x, 4)
        y = round(self.leg3_orientation.y, 4)
        z = round(self.leg3_orientation.z, 4)
        w = round(self.leg3_orientation.w, 4)
        self.last_recieved_stamp = rospy.Time.now()

        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])

        del_3 = sqrt(abs(0.2484**2 - (self.leg3z)**2))
        self.feet_3_x = self.leg3x + cos(yaw)*del_3
        self.feet_3_y = self.leg3y + sin(yaw)*del_3


    def update_leg4_pose(self, data):
        self.leg4_pose = data.pose[13].position #leg 4 link 3 is [13]
        self.leg4x = round(self.leg4_pose.x, 4)
        self.leg4y = round(self.leg4_pose.y, 4)
        self.leg4z = round(self.leg4_pose.z, 4)
        self.last_recieved_stamp = rospy.Time.now()

        self.leg4_orientation = data.pose[13].orientation
        x = round(self.leg4_orientation.x, 4)
        y = round(self.leg4_orientation.y, 4)
        z = round(self.leg4_orientation.z, 4)
        w = round(self.leg4_orientation.w, 4)
        self.last_recieved_stamp = rospy.Time.now()

        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])

        del_4 = sqrt(abs(0.2484**2 - (self.leg4z)**2))
        self.feet_4_x = self.leg4x + cos(yaw)*del_4
        self.feet_4_y = self.leg4y + sin(yaw)*del_4

    def move_joints1(self,joints_array):     ##return joint value for publisher
        i = 0
        self.c = self.c + 1
        for publisher_object in self.publishers_array1:
            joint_value = Float64()
            joint_value.data =joints_array[i]
            rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

    def move_joints2(self,joints_array):     ##return joint value for publisher
        i = 0
        for publisher_object in self.publishers_array2:
            joint_value = Float64()
            joint_value.data =joints_array[i]
            rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

    def move_joints3(self,joints_array):     ##return joint value for publisher
        i = 0
        for publisher_object in self.publishers_array3:
            joint_value = Float64()
            joint_value.data =joints_array[i]
            rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

    def move_joints4(self,joints_array):     ##return joint value for publisher
        i = 0
        for publisher_object in self.publishers_array4:
            joint_value = Float64()
            joint_value.data =joints_array[i]
            rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

    def update_joint_list(self,c,L1j,L2j,L3j,L4j):
        self.L1J = (np.append(self.L1J,L1j)).reshape(c,3)
        self.L2J = (np.append(self.L2J,L2j)).reshape(c,3)
        self.L3J = (np.append(self.L3J,L3j)).reshape(c,3)
        self.L4J = (np.append(self.L4J,L4j)).reshape(c,3)

    def update_pose_list(self,c,Re_x,Re_y,Ac_x,Ac_y,time):
    #def update_pose_list(self,c,Re_x,Re_y,Ac_x,Ac_y):
        self.Re_x = (np.append(self.Re_x,Re_x)).reshape(c,1)
        self.Re_y = (np.append(self.Re_y,Re_y)).reshape(c,1)
        self.Ac_x = (np.append(self.Ac_x,Ac_x)).reshape(c,1)
        self.Ac_y = (np.append(self.Ac_y,Ac_y)).reshape(c,1)
        self.t = (np.append(self.t,time)).reshape(c,1)

    def update_current_and_goal_pos(self,d,current_and_goal):
        self.curr_n_goal = (np.append(self.curr_n_goal,current_and_goal)).reshape(d,4)

    def update_orientation_list(self,c,Roll,Pitch,Yaw):
        self.Roll = (np.append(self.Roll,Roll)).reshape(c,1)
        self.Pitch = (np.append(self.Pitch,Pitch)).reshape(c,1)
        self.Yaw = (np.append(self.Yaw,Yaw)).reshape(c,1)

    def distance(self,goal_pose):
        x_distance = goal_pose.position.x - self.pose.x
        y_distance = goal_pose.position.y - self.pose.y
        distance = sqrt(x_distance**2+y_distance**2)
        return distance

    def start_loop(self,rate_value = 2.0):    ##assign position value according to time frame
        rospy.sleep(0.1)
        init_pos = self.pose
        rospy.loginfo(str(init_pos))
        rospy.sleep(0.1)
        goal_pose = Pose()
        goal_pose.position.x = input("Set your x goal: ")
        goal_pose.position.y = input("Set your y goal: ")
        goal_pose.position.z = input("Set your z goal: ")
        self.st = time.clock()
        self.start_time = time.time()

        targetArrived = False
        fs = 0
        count = 0
        fs_loop = 0
        X_com = []
        Y_com = []
        while count == 0:
            #plt.close()
            if not targetArrived and not rospy.is_shutdown():
                x_distance = goal_pose.position.x - self.pose.x
                y_distance = goal_pose.position.y - self.pose.y
                target_dir = atan2(y_distance,x_distance)
                print("Target direction:")
                print(target_dir)
                yaw = self.update_euler_angle(self.orientation,2)
                print("Base Orientation,yaw:")
                print(yaw)

                dir_diff = target_dir - (yaw + pi/2)
                if dir_diff <=-pi:
                    dir_diff = dir_diff + 2*pi
                else:
                    dir_diff = dir_diff
                print("Angle different of base from target:")
                print(dir_diff)

                # direction defining
                if dir_diff <=pi/4 and dir_diff >=-pi/4:
                    dir_diff = 8 #walking toward front
                elif dir_diff<= 0.75*pi and dir_diff >= pi/4:
                    dir_diff = 4 #walking toward left side
                elif dir_diff<=-pi/4 and dir_diff >= -0.75*pi:
                    dir_diff = 6  # walking toward right side
                else:
                    dir_diff = 2 # walking toward backs side

                if fs_loop == 0: ## checking directional from previous step if is same, continue the walk else change different directional walking loop
                    dir_diff_1 = dir_diff
                else:
                    if dir_diff != dir_diff_1: # reset the loop number if the directional need to be change
                        fs_loop = 0
                    else:
                        fs_loop = fs_loop

                #####start diredtional defination#### First, try to walking with no turning~
                if dir_diff == 8 : #WALK TO FRONT
                    print("The loop number after OP: {}".format(fs_loop))
                    print("The robot is walking forward with the sequence of 1-3-2-4.")
                    if fs_loop == 0:
                        loop = 0
                    else:
                        loop = None

                    c_x = self.pose.x
                    c_y = self.pose.y
                    c_z = self.pose.z
                    print("The Base position: ")
                    print(c_x,c_y,c_z)
                    #rospy.sleep(0.1)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print(Orient)

                    print("The yaw angle of Base")
                    yaw = self.update_euler_angle(self.orientation,2)
                    print(yaw)
                    rospy.sleep(0.01)
                    # fs_predefine
                    step_0 = FS_define0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 1 -3 - 2 - 4
                    fs_loop = fs_loop + 1
                    step_1 = FS_define0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 1 -3 - 2 - 4

                    if fs_loop-1 == 0: #remain the continuos stance leg's position
                        # fs_predefine
                        step_0 = step_0
                        step_1 = step_1
                    else:
                        if (fs_loop-1)%4 == 0:
                            P4_x = step_0[0][0]
                            P4_y = step_0[1][0]
                            step_0[0][1] = self.feet_3_x
                            step_0[1][1] = self.feet_3_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y

                            step_1[0][0] = self.feet_4_x
                            step_1[1][0] = self.feet_4_y
                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][3] = self.feet_4_x
                            step_1[1][3] = self.feet_4_y

                        elif (fs_loop-1)%4 == 1:
                            P1_x = step_0[0][1]
                            P1_y = step_0[1][1]
                            step_0[0][0] = self.feet_4_x
                            step_0[1][0] = self.feet_4_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y
                            step_0[0][3] = self.feet_4_x
                            step_0[1][3] = self.feet_4_y

                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][2] = self.feet_4_x
                            step_1[1][2] = self.feet_4_y

                        elif (fs_loop-1)%4 == 2:
                            P3_x = step_0[0][0]
                            P3_y = step_0[1][0]
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][2] = self.feet_4_x
                            step_0[1][2] = self.feet_4_y

                            step_1[0][0] = self.feet_3_x
                            step_1[1][0] = self.feet_3_y
                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][3] = self.feet_3_x
                            step_1[1][3] = self.feet_3_y
                        else:
                            P2_x = step_0[0][2]
                            P2_y = step_0[1][2]
                            step_0[0][0] = self.feet_3_x
                            step_0[1][0] = self.feet_3_y
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][3] = self.feet_3_x
                            step_0[1][3] = self.feet_3_y

                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][1] = self.feet_3_x
                            step_1[1][1] = self.feet_3_y

                    # add attribute of fs_predefined in gurobi_op for optimizing com trajectory
                    fs = np.append(step_0,step_1,1)
                    start_op = QPBodyTrajectory()
                    setattr(start_op,'fs',fs)
                    Xcom , Ycom = start_op.run_qp_model(fs)
                    X_com = np.append(X_com,Xcom)
                    Y_com = np.append(Y_com,Ycom)
                    #fig, ax = plt.subplots()
                    #self.plot_trajectory(X_com,Y_com,'r-')
                    #plt.pause(0.5)
                    #rospy.sleep(0.1)
                    # generate end-effector trajectory based on the Optimized COM Solution
                    fs_loop_prior = fs_loop-1
                    fs_check = (fs_loop-1) % 4
                    EF_P = []
                    if fs_check == 0:
                        #stance foot position should not be change!!
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][1]
                        P3_y = step_0[1][1]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]
                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qO1 = np.zeros((60,1))
                        qO2 = np.zeros((60,1))
                        qO3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qO1[i] = -(pi/8)*k_1[i]
                            qO2[i] = -0.523*k_2[i]
                            qO3[i] = -0.261*k_2[i]

                        pos1 = np.column_stack((qO1.reshape(60,1),qO2.reshape(60,1),qO3.reshape(60,1)))

                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1[i])
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.d = self.d +1
                            current_and_goal = np.column_stack((self.pose.x,self.pose.y,goal_pose.position.x,goal_pose.position.y))
                            self.update_current_and_goal_pos(self.d,current_and_goal)
                            self.update_joint_list(self.c,pos1[i],pos2,pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 1:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qTH1 = np.zeros((60,1))
                        qTH2 = np.zeros((60,1))
                        qTH3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qTH1[i] = 0.785*k_1[i]
                            qTH2[i] = -0.523*k_2[i]
                            qTH3[i] = -0.261*k_2[i]

                        pos3 = np.column_stack((qTH1.reshape(60,1),qTH2.reshape(60,1),qTH3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3[i])
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3[i],pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 2:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P4_x = step_0[0][2]
                        P4_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qT1 = np.zeros((60,1))
                        qT2 = np.zeros((60,1))
                        qT3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qT1[i] = (pi/8)*k_1[i]
                            qT2[i] = -0.523*k_2[i]
                            qT3[i] = -0.261*k_2[i]

                        pos2 = np.column_stack((qT1.reshape(60,1),qT2.reshape(60,1),qT3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2[i])
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2[i],pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    else:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qF1 = np.zeros((60,1))
                        qF2 = np.zeros((60,1))
                        qF3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qF1[i] = -0.785*k_1[i]
                            qF2[i] = -0.523*k_2[i]
                            qF3[i] = -0.261*k_2[i]

                        pos4 = np.column_stack((qF1.reshape(60,1),qF2.reshape(60,1),qF3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4[i])
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3,pos4[i])
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    print("Optimized COM :")
                    print(Xcom[59],Ycom[59])
                    dir_diff_1 = dir_diff

                elif dir_diff == 4: #WALK TO LEFT SIDE
                    print("The loop number after OP: {}".format(fs_loop))
                    print("The robot is walking to left with the sequence of 4-2-1-3.")
                    if fs_loop == 0:
                        loop = 0
                    else:
                        loop = None

                    c_x = self.pose.x
                    c_y = self.pose.y
                    c_z = self.pose.z
                    print("The Base position: ")
                    print(c_x,c_y,c_z)
                    #rospy.sleep(0.1)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print(Orient)

                    print("The yaw angle of Base")
                    yaw = self.update_euler_angle(self.orientation,2)
                    print(yaw)
                    rospy.sleep(0.1)
                    # fs_predefine
                    step_0 = FS_define_L0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 4 - 2 - 1 - 3
                    fs_loop = fs_loop + 1
                    step_1 = FS_define_L0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 4 - 2 - 1 - 3

                    if fs_loop-1 == 0: #remain the continuos stance leg's position
                        # fs_predefine
                        step_0 = step_0
                        step_1 = step_1
                    else:
                        if (fs_loop-1)%4 == 0:
                            P3_x = step_0[0][1]
                            P3_y = step_0[1][1]
                            step_0[0][0] = self.feet_1_x
                            step_0[1][0] = self.feet_1_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y
                            step_0[0][3] = self.feet_1_x
                            step_0[1][3] = self.feet_1_y

                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][2] = self.feet_3_x
                            step_1[1][2] = self.feet_3_y

                        elif (fs_loop-1)%4 == 1:
                            P4_x = step_0[0][0]
                            P4_y = step_0[1][0]
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][2] = self.feet_3_x
                            step_0[1][2] = self.feet_3_y

                            step_1[0][0] = self.feet_3_x
                            step_1[1][0] = self.feet_3_y
                            step_1[0][2] = self.feet_4_x
                            step_1[1][2] = self.feet_4_y
                            step_1[0][3] = self.feet_3_x
                            step_1[1][3] = self.feet_3_y

                        elif (fs_loop-1)%4 == 2:
                            P2_x = step_0[0][1]
                            P2_y = step_0[1][1]
                            step_0[0][0] = self.feet_3_x
                            step_0[1][0] = self.feet_3_y
                            step_0[0][2] = self.feet_4_x
                            step_0[1][2] = self.feet_4_y
                            step_0[0][3] = self.feet_3_x
                            step_0[1][3] = self.feet_3_y

                            step_1[0][0] = self.feet_4_x
                            step_1[1][0] = self.feet_4_y
                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][3] = self.feet_4_x
                            step_1[1][3] = self.feet_4_y
                        else:
                            P1_x = step_0[0][1]
                            P1_y = step_0[1][1]
                            step_0[0][0] = self.feet_4_x
                            step_0[1][0] = self.feet_4_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y
                            step_0[0][3] = self.feet_4_x
                            step_0[1][3] = self.feet_4_y

                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][0] = self.feet_1_x
                            step_1[1][0] = self.feet_1_y
                            step_1[0][3] = self.feet_1_x
                            step_1[1][3] = self.feet_1_y

                    # add attribute of fs_predefined in gurobi_op for optimizing com trajectory
                    fs = np.append(step_0,step_1,1)
                    start_op = QPBodyTrajectory()
                    setattr(start_op,'fs',fs)
                    Xcom , Ycom = start_op.run_qp_model(fs)
                    X_com = np.append(X_com,Xcom)
                    Y_com = np.append(Y_com,Ycom)
                    #fig, ax = plt.subplots()
                    #self.plot_trajectory(X_com,Y_com,'r-')
                    #plt.pause(0.5)
                    #rospy.sleep(1)
                    # generate end-effector trajectory based on the Optimized COM Solution
                    fs_loop_prior = fs_loop-1
                    fs_check = (fs_loop-1) % 4
                    EF_P = []
                    if fs_check == 0:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][0]
                        P1_y = step_0[1][0]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][1]
                        P3_y = step_0[1][1]
                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qF1 = np.zeros((60,1))
                        qF2 = np.zeros((60,1))
                        qF3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qF1[i] = -(pi/8)*k_1[i]
                            qF2[i] = -0.523*k_2[i]
                            qF3[i] = -0.261*k_2[i]

                        pos4 = np.column_stack((qF1.reshape(60,1),qF2.reshape(60,1),qF3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4[i])
                            self.b = self.b +1
                            self.d = self.d +1
                            current_and_goal = np.column_stack((self.pose.x,self.pose.y,goal_pose.position.x,goal_pose.position.y))
                            self.update_current_and_goal_pos(self.d,current_and_goal)
                            self.update_joint_list(self.c,pos1,pos2,pos3,pos4[i])
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 1:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P3_x = step_0[0][2]
                        P3_y = step_0[1][2]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qT1 = np.zeros((60,1))
                        qT2 = np.zeros((60,1))
                        qT3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qT1[i] = 0.785*k_1[i]
                            qT2[i] = -0.523*k_2[i]
                            qT3[i] = -0.261*k_2[i]

                        pos2 = np.column_stack((qT1.reshape(60,1),qT2.reshape(60,1),qT3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2[i])
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2[i],pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 2:
                        #stance foot position should not be change!!
                        P2_x = step_0[0][1]
                        P2_y = step_0[1][1]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]
                        P4_x = step_0[0][2]
                        P4_y = step_0[1][2]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qO1 = np.zeros((60,1))
                        qO2 = np.zeros((60,1))
                        qO3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qO1[i] = (pi/8)*k_1[i]
                            qO2[i] = -0.523*k_2[i]
                            qO3[i] = -0.261*k_2[i]

                        pos1 = np.column_stack((qO1.reshape(60,1),qO2.reshape(60,1),qO3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1[i])
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1[i],pos2,pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    else:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qTH1 = np.zeros((60,1))
                        qTH2 = np.zeros((60,1))
                        qTH3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qTH1[i] = -0.785*k_1[i]
                            qTH2[i] = -0.523*k_2[i]
                            qTH3[i] = -0.261*k_2[i]

                        pos3 = np.column_stack((qTH1.reshape(60,1),qTH2.reshape(60,1),qTH3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3[i])
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3[i],pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    print("Optimized COM :")
                    print(Xcom[59],Ycom[59])
                    dir_diff_1 = dir_diff

                elif dir_diff == 6: # WALK TO RIGHT SIDE
                    print("The loop number after OP: {}".format(fs_loop))
                    print("The robot is walking to right with the sequence of 2-4-3-1.")
                    if fs_loop == 0:
                        loop = 0
                    else:
                        loop = None

                    c_x = self.pose.x
                    c_y = self.pose.y
                    c_z = self.pose.z
                    print("The Base position: ")
                    print(c_x,c_y,c_z)
                    #rospy.sleep(0.1)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print(Orient)

                    print("The yaw angle of Base")
                    yaw = self.update_euler_angle(self.orientation,2)
                    print(yaw)
                    rospy.sleep(0.01)
                    # fs_predefine
                    step_0 = FS_define_R0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 2 - 4 - 3 - 1
                    fs_loop = fs_loop + 1
                    step_1 = FS_define_R0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 2 - 4 - 3 - 1

                    if fs_loop-1 == 0: #remain the continuos stance leg's position
                        # fs_predefine
                        step_0 = step_0
                        step_1 = step_1
                    else:
                        if (fs_loop-1)%4 == 0:
                            P1_x = step_0[0][2]
                            P1_y = step_0[1][2]
                            step_0[0][1] = self.feet_3_x
                            step_0[1][1] = self.feet_3_y
                            step_0[0][0] = self.feet_4_x
                            step_0[1][0] = self.feet_4_y
                            step_0[0][3] = self.feet_4_x
                            step_0[1][3] = self.feet_4_y

                            step_1[0][0] = self.feet_3_x
                            step_1[1][0] = self.feet_3_y
                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][3] = self.feet_3_x
                            step_1[1][3] = self.feet_3_y

                        elif (fs_loop-1)%4 == 1:
                            P2_x = step_0[0][2]
                            P2_y = step_0[1][2]
                            step_0[0][0] = self.feet_3_x
                            step_0[1][0] = self.feet_3_y
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][3] = self.feet_3_x
                            step_0[1][3] = self.feet_3_y

                            step_1[0][0] = self.feet_2_x
                            step_1[1][0] = self.feet_2_y
                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][3] = self.feet_2_x
                            step_1[1][3] = self.feet_2_y

                        elif (fs_loop-1)%4 == 2:
                            P4_x = step_0[0][2]
                            P4_y = step_0[1][2]
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][0] = self.feet_2_x
                            step_0[1][0] = self.feet_2_y
                            step_0[0][3] = self.feet_2_x
                            step_0[1][3] = self.feet_2_y

                            step_1[0][1] = self.feet_4_x
                            step_1[1][1] = self.feet_4_y
                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y

                        else:
                            P3_x = step_0[0][0]
                            P3_y = step_0[1][0]
                            step_0[0][1] = self.feet_4_x
                            step_0[1][1] = self.feet_4_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y

                            step_1[0][0] = self.feet_4_x
                            step_1[1][0] = self.feet_4_y
                            step_1[0][1] = self.feet_3_x
                            step_1[1][1] = self.feet_3_y
                            step_1[0][3] = self.feet_4_x
                            step_1[1][3] = self.feet_4_y

                    # add attribute of fs_predefined in gurobi_op for optimizing com trajectory
                    fs = np.append(step_0,step_1,1)
                    start_op = QPBodyTrajectory()
                    setattr(start_op,'fs',fs)
                    Xcom , Ycom = start_op.run_qp_model(fs)
                    X_com = np.append(X_com,Xcom)
                    Y_com = np.append(Y_com,Ycom)
                    #fig, ax = plt.subplots()
                    #self.plot_trajectory(X_com,Y_com,'r-')
                    #plt.pause(0.5)
                    #rospy.sleep(1)
                    # generate end-effector trajectory based on the Optimized COM Solution
                    fs_loop_prior = fs_loop-1
                    fs_check = (fs_loop-1) % 4
                    EF_P = []
                    if fs_check == 0:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][2]
                        P1_y = step_0[1][2]
                        P3_x = step_0[0][1]
                        P3_y = step_0[1][1]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]
                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qT1 = np.zeros((60,1))
                        qT2 = np.zeros((60,1))
                        qT3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qT1[i] = -(pi/8)*k_1[i]
                            qT2[i] = -0.523*k_2[i]
                            qT3[i] = -0.261*k_2[i]

                        pos2 = np.column_stack((qT1.reshape(60,1),qT2.reshape(60,1),qT3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2[i])
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.d = self.d +1
                            current_and_goal = np.column_stack((self.pose.x,self.pose.y,goal_pose.position.x,goal_pose.position.y))
                            self.update_current_and_goal_pos(self.d,current_and_goal)
                            self.update_joint_list(self.c,pos1,pos2[i],pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 1:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qF1 = np.zeros((60,1))
                        qF2 = np.zeros((60,1))
                        qF3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qF1[i] = 0.785*k_1[i]
                            qF2[i] = -0.523*k_2[i]
                            qF3[i] = -0.261*k_2[i]

                        pos4 = np.column_stack((qF1.reshape(60,1),qF2.reshape(60,1),qF3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4[i])
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3,pos4[i])
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 2:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P4_x = step_0[0][2]
                        P4_y = step_0[1][2]
                        P2_x = step_0[0][0]
                        P2_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qTH1 = np.zeros((60,1))
                        qTH2 = np.zeros((60,1))
                        qTH3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qTH1[i] = (pi/8)*k_1[i]
                            qTH2[i] = -0.523*k_2[i]
                            qTH3[i] = -0.261*k_2[i]

                        pos3 = np.column_stack((qTH1.reshape(60,1),qTH2.reshape(60,1),qTH3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3[i])
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3[i],pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    else:
                        #stance foot position should not be change!!
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]
                        P4_x = step_0[0][1]
                        P4_y = step_0[1][1]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qO1 = np.zeros((60,1))
                        qO2 = np.zeros((60,1))
                        qO3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qO1[i] = -0.785*k_1[i]
                            qO2[i] = -0.523*k_2[i]
                            qO3[i] = -0.261*k_2[i]

                        pos1 = np.column_stack((qO1.reshape(60,1),qO2.reshape(60,1),qO3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1[i])
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1[i],pos2,pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    print("Optimized COM :")
                    print(Xcom[59],Ycom[59])
                    dir_diff_1 = dir_diff

                else:  ## walking back ward
                    print("The loop number after OP: {}".format(fs_loop))
                    print("The robot is walking backward with the sequence of 3-1-4-2.")
                    if fs_loop == 0:
                        loop = 0
                    else:
                        loop = None

                    c_x = self.pose.x
                    c_y = self.pose.y
                    c_z = self.pose.z
                    print("The Base position: ")
                    print(c_x,c_y,c_z)
                    #rospy.sleep(0.1)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print(Orient)

                    print("The yaw angle of Base")
                    yaw = self.update_euler_angle(self.orientation,2)
                    print(yaw)
                    rospy.sleep(0.01)
                    # fs_predefine
                    step_0 = FS_define_B0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 3 - 1 - 4 - 2
                    fs_loop = fs_loop + 1
                    step_1 = FS_define_B0(c_x,c_y,c_z,Orient,fs_loop,loop) #sequence: 3 - 1 - 4 - 2

                    if fs_loop-1 == 0: #remain the continuos stance leg's position
                        # fs_predefine
                        step_0 = step_0
                        step_1 = step_1
                    else:
                        if (fs_loop-1)%4 == 0:
                            P2_x = step_0[0][2]
                            P2_y = step_0[1][2]
                            step_0[0][0] = self.feet_4_x
                            step_0[1][0] = self.feet_4_y
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][3] = self.feet_4_x
                            step_0[1][3] = self.feet_4_y

                            step_1[0][0] = self.feet_4_x
                            step_1[1][0] = self.feet_4_y
                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][3] = self.feet_4_x
                            step_1[1][3] = self.feet_4_y

                        elif (fs_loop-1)%4 == 1:
                            P3_x = step_0[0][1]
                            P3_y = step_0[1][1]
                            step_0[0][0] = self.feet_4_x
                            step_0[1][0] = self.feet_4_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y
                            step_0[0][3] = self.feet_4_x
                            step_0[1][3] = self.feet_4_y

                            step_1[0][0] = self.feet_3_x
                            step_1[1][0] = self.feet_3_y
                            step_1[0][2] = self.feet_2_x
                            step_1[1][2] = self.feet_2_y
                            step_1[0][3] = self.feet_3_x
                            step_1[1][3] = self.feet_3_y

                        elif (fs_loop-1)%4 == 2:
                            P1_x = step_0[0][1]
                            P1_y = step_0[1][1]
                            step_0[0][0] = self.feet_3_x
                            step_0[1][0] = self.feet_3_y
                            step_0[0][2] = self.feet_2_x
                            step_0[1][2] = self.feet_2_y
                            step_0[0][3] = self.feet_3_x
                            step_0[1][3] = self.feet_3_y

                            step_1[0][0] = self.feet_3_x
                            step_1[1][0] = self.feet_3_y
                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][3] = self.feet_3_x
                            step_1[1][3] = self.feet_3_y
                        else:
                            P4_x = step_0[0][2]
                            P4_y = step_0[1][2]
                            step_0[0][0] = self.feet_3_x
                            step_0[1][0] = self.feet_3_y
                            step_0[0][1] = self.feet_1_x
                            step_0[1][1] = self.feet_1_y
                            step_0[0][3] = self.feet_3_x
                            step_0[1][3] = self.feet_3_y

                            step_1[0][0] = self.feet_4_x
                            step_1[1][0] = self.feet_4_y
                            step_1[0][1] = self.feet_1_x
                            step_1[1][1] = self.feet_1_y
                            step_1[0][3] = self.feet_4_x
                            step_1[1][3] = self.feet_4_y

                    # add attribute of fs_predefined in gurobi_op for optimizing com trajectory
                    fs = np.append(step_0,step_1,1)
                    start_op = QPBodyTrajectory()
                    setattr(start_op,'fs',fs)
                    Xcom , Ycom = start_op.run_qp_model(fs)
                    X_com = np.append(X_com,Xcom)
                    Y_com = np.append(Y_com,Ycom)
                    #fig, ax = plt.subplots()
                    #self.plot_trajectory(X_com,Y_com,'r-')
                    #plt.pause(0.5)
                    #rospy.sleep(1)
                    # generate end-effector trajectory based on the Optimized COM Solution
                    fs_loop_prior = fs_loop-1
                    fs_check = (fs_loop-1) % 4
                    EF_P = []
                    if fs_check == 0:
                        #stance foot position should not be change!!
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]
                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qTH1 = np.zeros((60,1))
                        qTH2 = np.zeros((60,1))
                        qTH3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qTH1[i] = -(pi/8)*k_1[i]
                            qTH2[i] = -0.523*k_2[i]
                            qTH3[i] = -0.261*k_2[i]

                        pos3 = np.column_stack((qTH1.reshape(60,1),qTH2.reshape(60,1),qTH3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3[i])
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.d = self.d +1
                            current_and_goal = np.column_stack((self.pose.x,self.pose.y,goal_pose.position.x,goal_pose.position.y))
                            self.update_current_and_goal_pos(self.d,current_and_goal)
                            self.update_joint_list(self.c,pos1,pos2,pos3[i],pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 1:
                        #stance foot position should not be change!!
                        P3_x = step_0[0][1]
                        P3_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P4_x = step_0[0][0]
                        P4_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qO1 = np.zeros((60,1))
                        qO2 = np.zeros((60,1))
                        qO3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qO1[i] = 0.785*k_1[i]
                            qO2[i] = -0.523*k_2[i]
                            qO3[i] = -0.261*k_2[i]

                        pos1 = np.column_stack((qO1.reshape(60,1),qO2.reshape(60,1),qO3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)

                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1[i])
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1[i],pos2,pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    elif fs_check == 2:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P2_x = step_0[0][2]
                        P2_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qF1 = np.zeros((60,1))
                        qF2 = np.zeros((60,1))
                        qF3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qF1[i] = (pi/8)*k_1[i]
                            qF2[i] = -0.523*k_2[i]
                            qF3[i] = -0.261*k_2[i]

                        pos4 = np.column_stack((qF1.reshape(60,1),qF2.reshape(60,1),qF3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #rospy.sleep(3.5)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos2 = IK_Leg2(Xcom[i],Ycom[i],c_z,Orient,P2_x,P2_y,0)
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2)
                            self.move_joints3(pos3)
                            self.move_joints4(pos4[i])
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2,pos3,pos4[i])
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    else:
                        #stance foot position should not be change!!
                        P1_x = step_0[0][1]
                        P1_y = step_0[1][1]
                        P4_x = step_0[0][2]
                        P4_y = step_0[1][2]
                        P3_x = step_0[0][0]
                        P3_y = step_0[1][0]

                        #swing leg trajectory plan:
                        ts = np.linspace(0,6,60)
                        k_1 = np.zeros((60,1))
                        k_2 = np.zeros((60,1))
                        qT1 = np.zeros((60,1))
                        qT2 = np.zeros((60,1))
                        qT3 = np.zeros((60,1))
                        for i in range (60):
                            k_1[i] = sin((ts[i]/6)*0.5*pi)
                            k_2[i] = (1-cos((ts[i]/6)*2*pi))*0.5
                            qT1[i] = -0.785*k_1[i]
                            qT2[i] = -0.523*k_2[i]
                            qT3[i] = -0.261*k_2[i]

                        pos2 = np.column_stack((qT1.reshape(60,1),qT2.reshape(60,1),qT3.reshape(60,1)))
                        Orient = self.update_euler_angle(self.orientation,3)
                        print(Orient)
                        #ik :
                        for i in range (60):
                            #Orient = self.update_euler_angle(self.orientation,3)
                            #c_z = self.pose.z
                            pos1 = IK_Leg1(Xcom[i],Ycom[i],c_z,Orient,P1_x,P1_y,0)  #using the optimize COM trajectory to generate joint trajectories!!
                            pos3 = IK_Leg3(Xcom[i],Ycom[i],c_z,Orient,P3_x,P3_y,0)
                            pos4 = IK_Leg4(Xcom[i],Ycom[i],c_z,Orient,P4_x,P4_y,0)
                            self.time = time.clock() - self.st
                            self.move_joints1(pos1)
                            self.move_joints2(pos2[i])
                            self.move_joints3(pos3)
                            self.move_joints4(pos4)
                            self.b = self.b +1
                            self.update_joint_list(self.c,pos1,pos2[i],pos3,pos4)
                            self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y,self.time)
                            #self.update_pose_list(self.b,Xcom[i],Ycom[i],self.pose.x,self.pose.y)
                            roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                            self.update_orientation_list(self.b,roll,pitch,yaw)
                            #print("COM reference: {},{}".format(Xcom[i],Ycom[i]))
                            #print("Actual COM: {},{}".format(self.pose.x,self.pose.y))
                            #x_err = self.pose.x-Xcom[i]
                            #y_err = self.pose.y-Ycom[i]
                            #print("error different: {},{}".format(x_err,y_err))

                        #fig, ax = plt.subplots()
                        #self.plot_trajectory(self.Ac_x,self.Ac_y,'b-')
                        #plt.pause(0.5)

                    print("Optimized COM :")
                    print(Xcom[59],Ycom[59])
                    dir_diff_1 = dir_diff

                np.savetxt("L1_J.csv", self.L1J, delimiter=",")
                np.savetxt("L2_J.csv", self.L2J, delimiter=",")
                np.savetxt("L3_J.csv", self.L3J, delimiter=",")
                np.savetxt("L4_J.csv", self.L4J, delimiter=",")
                np.savetxt("input_label.csv", self.curr_n_goal, delimiter=",")

                np.savetxt("RE_COM_POSE_X.csv", self.Re_x, delimiter=",") # optimized COM_X
                np.savetxt("RE_COM_POSE_Y.csv", self.Re_y, delimiter=",") # optimized COM_Y
                np.savetxt("AC_COM_POSE_X.csv", self.Ac_x, delimiter=",") # measured com_x
                np.savetxt("AC_COM_POSE_Y.csv", self.Ac_y, delimiter=",") # measured com_y
                np.savetxt("ROLL.csv", self.Roll, delimiter=",") # roll
                np.savetxt("PITCH.csv", self.Pitch, delimiter=",") # pitch
                np.savetxt("YAW.csv", self.Yaw, delimiter=",") # yaw
                np.savetxt("Time.csv", self.t, delimiter = ",")#time

                if (self.distance(goal_pose)<= distance_tolerance):
                    print("Target is arrived")
                    print("The current position is:")
                    Xnow = self.pose.x
                    Ynow = self.pose.y
                    print(Xnow,Ynow)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print("Actual COM orientation,(roll,pitch,yaw):")
                    print(Orient)
                    self.end_time = time.time()
                    runtime = self.end_time - self.start_time
                    print('Runtime of the program is: ' + str(runtime) + 'seconds')
                    targetArrived = True
                    count = 1

                else:
                    #rospy.sleep(0.1)
                    Xnow = self.pose.x
                    Ynow = self.pose.y
                    print("Actual COM position:")
                    print(Xnow,Ynow)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print("Actual COM orientation,(roll,pitch,yaw):")
                    print(Orient)
                    print("Distance to reach the Goal:")
                    print(str(self.distance(goal_pose)))



if __name__ =="__main__":                       ##initiate the main coding
    joint_publisher = JointPub()                ## assign the class name as joint_publisher
    rate_value = 1.0                            ## define the rate value
    joint_publisher.start_loop(rate_value)      ## assign the rate value to one of the component of the class
