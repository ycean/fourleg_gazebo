#!/usr/bin/env python
#THIS IS SCRIPT IS THE MODIFIED VERSION OF  JP_COM_QP_OMNI_DIR_FV3_v2.py
#ALL THE JOINT ANGLE OF THE LEG ARE TRAINED

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
import Train_Joint_model5000
from Train_Joint_model5000 import*


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
        self.imu_subscriber = rospy.Subscriber('/fourleg/imu/data',Imu, self.update_orientation)

        self.pose = LinkStates()
        self.orientation = Imu()

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

    def update_pose_list(self,c,Ac_x,Ac_y,time):
    #def update_pose_list(self,c,Ac_x,Ac_y):
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
                pos1 = predict_L1J(self.pose.x, self.pose.y, goal_pose.position.x, goal_pose.position.y)
                pos2 = predict_L2J(self.pose.x, self.pose.y, goal_pose.position.x, goal_pose.position.y)
                pos3 = predict_L3J(self.pose.x, self.pose.y, goal_pose.position.x, goal_pose.position.y)
                pos4 = predict_L4J(self.pose.x, self.pose.y, goal_pose.position.x, goal_pose.position.y)

                for i in range (240):
                    self.move_joints1(pos1[i])
                    self.move_joints2(pos2[i])
                    self.move_joints3(pos3[i])
                    self.move_joints4(pos4[i])
                    self.time = time.clock() - self.st
                    self.b = self.b +1
                    #self.d = self.d +1
                    #current_and_goal = np.column_stack((self.pose.x,self.pose.y,goal_pose.position.x,goal_pose.position.y))
                    #self.update_current_and_goal_pos(self.d,current_and_goal)
                    self.update_joint_list(self.c,pos1[i],pos2[i],pos3[i],pos4[i])
                    self.update_pose_list(self.b,self.pose.x,self.pose.y,self.time)
                    roll,pitch,yaw = self.update_euler_angle(self.orientation,3)
                    self.update_orientation_list(self.c,roll,pitch,yaw)

                np.savetxt("LEG1_JOINTS.csv", self.L1J, delimiter=",")
                np.savetxt("LEG2_JOINTS.csv", self.L2J, delimiter=",")
                np.savetxt("LEG3_JOINTS.csv", self.L3J, delimiter=",")
                np.savetxt("LEG4_JOINTS.csv", self.L4J, delimiter=",")
                np.savetxt("SIM_WITH_ML_COM_POSE_X.csv", self.Ac_x, delimiter=",") # measured com_x
                np.savetxt("SIM_WITH_ML_COM_POSE_Y.csv", self.Ac_y, delimiter=",") # measured com_y
                np.savetxt("ROLL.csv", self.Roll, delimiter=",") # roll
                np.savetxt("PITCH.csv", self.Pitch, delimiter=",") # pitch
                np.savetxt("YAW.csv", self.Yaw, delimiter=",") # yaw
                np.savetxt("Time.csv", self.t, delimiter = ",")#time
                self.end_time = time.time()
                runtime = self.end_time - self.start_time

                if (self.distance(goal_pose)<= distance_tolerance):
                    print("Target is arrived")
                    print("The current position is:")
                    Xnow = self.pose.x
                    Ynow = self.pose.y
                    print(Xnow,Ynow)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print("Actual COM orientation,(roll,pitch,yaw):")
                    print(Orient)
                    print('Runtime of the program is: ' + str(runtime) + 'seconds')
                    targetArrived = True
                    count = 1

                elif runtime >= 250:
                    print("Runtime limit is reach !")
                    print("The current position is:")
                    Xnow = self.pose.x
                    Ynow = self.pose.y
                    print(Xnow,Ynow)
                    Orient = self.update_euler_angle(self.orientation,3)
                    print("Actual COM orientation,(roll,pitch,yaw):")
                    print(Orient)
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
