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
from time import time

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

    def __init__(self, verbose=False):
        self.verbose = verbose
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7") # link twist
        q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # joint angles
        
        # Create Modified DH parameters
        DH_Table = {    alpha0:    0.0,    a0:       0.0,     d1:     0.75,     q1:         q1,
                        alpha1: -pi/2.,    a1:      0.35,     d2:      0.0,     q2: -pi/2 + q2,
                        alpha2:    0.0,    a2:      1.25,     d3:      0.0,     q3:         q3,
                        alpha3: -pi/2.,    a3:    -0.054,     d4:      1.5,     q4:         q4,
                        alpha4:  pi/2.,    a4:       0.0,     d5:      0.0,     q5:         q5,
                        alpha5: -pi/2.,    a5:       0.0,     d6:      0.0,     q6:         q6,
                        alpha6:    0.0,    a6:       0.0,     d7:    0.303,     q7:        0.0}

        # Create individual transformation matrices
        self.T0_1 =  self.__t__(alpha0, a0, d1, q1).subs(DH_Table)
        self.T1_2 =  self.__t__(alpha1, a1, d2, q2).subs(DH_Table)
        self.T2_3 =  self.__t__(alpha2, a2, d3, q3).subs(DH_Table) # Remaining matrices aren't needed. See junk code at bottom of file.
        self.T3_4 =  self.__t__(alpha3, a3, d4, q4).subs(DH_Table)
        self.T4_5 =  self.__t__(alpha4, a4, d5, q5).subs(DH_Table)
        self.T5_6 =  self.__t__(alpha5, a5, d6, q6).subs(DH_Table)
        self.T6_EE = self.__t__(alpha6, a6, d7, q7).subs(DH_Table)
        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE
        
        # find EE rotational matrix
        roll, pitch, yaw = symbols ('roll pitch yaw')
        px, py, pz = symbols ('px py pz')
        
        # Compensate for rotation discrepancy between DH parameters and Gazebo

        Rot_x = self.__rot_x__(roll)
        Rot_y = self.__rot_y__(pitch)
        Rot_z = self.__rot_z__(yaw)
        Rot_xyz = Rot_x * Rot_y * Rot_z
    
        # Rotation error (More information in the KR210 kinematics section)
        Rot_Error = Rot_z.subs(yaw, radians(180)) * Rot_y.subs(pitch, radians(-90))
        self.Rot_EE = Rot_xyz * Rot_Error
        self.EE = Matrix([[px], [py], [pz]])

        if self.verbose:
            print ("Symbolic matrices:")
            self.print_matrix(self.T0_1, "T0_1")
            self.print_matrix(self.T1_2, "T1_2")
            self.print_matrix(self.T2_3, "T2_3")
            self.print_matrix(self.T3_4, "T3_4")
            self.print_matrix(self.T4_5, "T4_5")
            self.print_matrix(self.T5_6, "T5_6")
            self.print_matrix(self.T6_EE, "T6_EE")
            self.print_matrix(self.T0_EE, "T0_EE")
            self.print_matrix(Rot_x, "Rot_x")
            self.print_matrix(Rot_y, "Rot_y")
            self.print_matrix(Rot_z, "Rot_z")
            self.print_matrix(Rot_xyz, "Rot_xyz")
            self.print_matrix(Rot_Error, "Rot_Error")
            self.print_matrix(self.Rot_EE, "Rot_EE")
        
    def print_matrix(self, matrix, name):
        print ("{} : ".format(name))
        pprint (matrix)
    
    def get_wc(self, px, py, pz, roll, pitch, yaw):
        # First get the position of the Wrist Center (WC)    
        Rot_EE = self.Rot_EE.subs({'roll': roll, 'pitch': pitch, 'yaw': yaw})
        EE = self.EE.subs({'px': px, 'py': py, 'pz': pz})
        WC = EE - (0.303) * Rot_EE[:,2]
        return WC
    
    def get_joint_angles(self, px, py, pz, roll, pitch, yaw):
        start_time = time()

        # First get the position of the Wrist Center (WC)    
        Rot_EE = self.Rot_EE.subs({'roll': roll, 'pitch': pitch, 'yaw': yaw})
        EE = self.EE.subs({'px': px, 'py': py, 'pz': pz})
        WC = EE - (0.303) * Rot_EE[:,2]

        # Position analysis (Calculate joint angles using Geometric IK method):
        theta1 = atan2(WC[1], WC[0])
        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
        side_c = 1.25

        angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

        theta2 = pi / 2.0 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
        theta3 = pi / 2.0 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

        # Orientation analysis (Euler angles from rotation matrix):
        R0_3 = self.T0_1[0:3, 0:3] * self.T1_2[0:3, 0:3] * self.T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
        R3_6 = R0_3.T * Rot_EE

        # The formulae below are obtained by
        # Check this link for optimization to avoid excessive rotations of the wrist:
        #   https://udacity-robotics.slack.com/archives/C5HUQ0HB9/p1499136717183191
        theta4 = atan2(R3_6[2, 2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
        theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
        
        joint_angles = [float(theta1), float(theta2), float(theta3), float(theta4), float(theta5), float(theta6)]
        
        end_time = time()
        
        if self.verbose:
            print ("Evaluated matrices:")
            self.print_matrix(Rot_EE, "Rot_EE")
            self.print_matrix(EE, "EE")
            self.print_matrix(WC, "WC")
            self.print_matrix(R0_3, "R0_3")
            self.print_matrix(R3_6, "R3_6")
            
        ## FK Error
#        if check_error:
#            ee = self.get_fk(*joint_angles)
#            print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (end_time-start_time))
#            ee_x_e = abs(ee[0]-px)
#            ee_y_e = abs(ee[1]-py)
#            ee_z_e = abs(ee[2]-pz)
#            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
#            print ("Expected end effector [{}, {}, {}]: ".format(px, py, pz))
#            print ("End effector error for x position is: %04.8f units" % ee_x_e)
#            print ("End effector error for y position is: %04.8f units" % ee_y_e)
#            print ("End effector error for z position is: %04.8f units" % ee_z_e)
#            print ("Overall end effector offset is: %04.8f units \n" % ee_offset)
        
        print ("Returning IK joint angles {} in {} seconds: ".format(joint_angles, end_time - start_time))
        return joint_angles

    def get_fk(self, theta1, theta2, theta3, theta4, theta5, theta6):
        return self.T0_EE.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3, 'q4': theta4, 'q5': theta5, 'q6': theta6})

