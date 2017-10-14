#!/usr/bin/env python

# Copyright (C) 2017 Safdar Kureishy
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Safdar Kureishy

# import modules
import tf
from mpmath import *
from sympy import *

class Kuka210IKSolver(object):
    #            
    # Define Modified DH Transformation matrix
    #
    def __t__(self, alpha, a, d, q):
        TF = Matrix([
                [           cos(q),              -sin(q),           0.0,                    a],
                [sin(q)*cos(alpha),    cos(q)*cos(alpha),   -sin(alpha),        -sin(alpha)*d],
                [sin(q)*sin(alpha),    cos(q)*sin(alpha),    cos(alpha),         cos(alpha)*d],
                [              0.0,                  0.0,           0.0,                  1.0],
            ])
        return TF

    def __rot_x__(self, roll):
        rotation = Matrix([
                        [1.0,          0.0,                0.0],
                        [0.0,     cos(roll),        -sin(roll)],
                        [0.0,     sin(roll),         cos(roll)],
                    ]) # roll
        return rotation

    def __rot_y__(self, pitch):
        rotation = Matrix([
                        [  cos(pitch),      0.0,         sin(pitch)],
                        [       0.0,        1.0,                0.0],
                        [ -sin(pitch),      0.0,         cos(pitch)],
                    ]) # pitch
        return rotation
   
    def __rot_z__(self, yaw):
        rotation = Matrix([
                        [  cos(yaw),     -sin(yaw),       0.0],
                        [  sin(yaw),      cos(yaw),       0.0],
                        [       0.0,           0.0,       1.0],
                    ]) # yaw
        return rotation

    def __init__(self):
        pass
        
    def get_joint_angles(self, px, py, pz, roll, pitch, yaw):
        ### Your FK code here
        # Create symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7") # link twist
        q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # joint angles

        #
        #   
        # Create Modified DH parameters
        #
        DH_Table = {    alpha0:    0.0,    a0:       0.0,     d1:     0.75,     q1:         q1,
                        alpha1: -pi/2.,    a1:      0.35,     d2:      0.0,     q2: -pi/2 + q2,
                        alpha2:    0.0,    a2:      1.25,     d3:      0.0,     q3:         q3,
                        alpha3: -pi/2.,    a3:    -0.054,     d4:      1.5,     q4:         q4,
                        alpha4:  pi/2.,    a4:       0.0,     d5:      0.0,     q5:         q5,
                        alpha5: -pi/2.,    a5:       0.0,     d6:      0.0,     q6:         q6,
                        alpha6:    0.0,    a6:       0.0,     d7:    0.303,     q7:        0.0}

        #
        # Create individual transformation matrices
        #
        T0_1 =  self.__t__(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 =  self.__t__(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 =  self.__t__(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 =  self.__t__(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 =  self.__t__(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 =  self.__t__(alpha5, a5, d6, q6).subs(DH_Table)
        T6_EE = self.__t__(alpha6, a6, d7, q7).subs(DH_Table)
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        ### Your IK code here 
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        #
        # find EE rotational matrix
        r, p, y = symbols ('r p y')

        Rot_x = self.__rot_x__(r)
        Rot_y = self.__rot_y__(p)
        Rot_z = self.__rot_z__(yaw)
        Rot_EE = Rot_x * Rot_y * Rot_z
    
        # Rotation error (More information in the KR210 kinematics section)
        Rot_Error = Rot_z.subs(y, radians(180)) * Rot_y.subs(p, radians(-90))

        Rot_EE = Rot_EE * Rot_Error
        Rot_EE = Rot_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
    
        EE = Matrix([[px],
                     [py],
                     [pz]])
        WC = EE - (0.303) * Rot_EE[:,2]

        #
        # Calculate joint angles using Geometric IK method
        #
        #
        theta1 = atan2(WC[1], WC[0])
        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
        side_c = 1.25

        angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

        theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
        theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        R3_6 = R0_3.inv("LU") * Rot_EE

        # Euler angles from rotation matrix
        theta4 = atan2(R3_6[2, 2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
        theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
        
        joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
        print ("Returning IK joint angles: ", joint_angles)
        return joint_angles


