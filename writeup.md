# Kuka KR210 Pick and Place Project
---

![Kuka KR210 Pick and Place Project][kuka_pick_and_place]

The purpose of this project is to gain exposure to Inverse Kinematic (IK) analysis (forward and inverse), for a simulated [Kuka KR210](https://www.kuka.com/en-us/products/robotics-systems industrial-robots/kr-210-2-f-exclusive) 6-DOF (Degree Of Freedom) manipulator, to grasp a can from a random position on a shelf, and drop it into a bin placed next to it.

The simulation, at a high level, consists of the following sequence of steps:
1. Move the manipulator (gripper) arm to a grasping position relative to the can on the shelf
1. Reach out to grasp the can
1. Move the manipulator (gripper) arm above the bin
1. Release the can to let it fall into the bin

The IK analysis is only required for steps 1 and 3 above. Steps 2 and 4 are supported rudimentary motions of this robot.

### Table of Contents

- [Denavit-Hartenberg Diagram](#denavit-hartenberg-diagram)
- [Denavit-Hartenberg (Modified) Parameters](#denavit-hartenberg-modified-parameters)
- [Transformation Matrices](#transformation-matrices)
- [Homogeneous Transformation Matrix](#homogeneous-transformation-matrix)
- [Inverse Kinematic Analysis](#inverse-kinematic-analysis)
    - [Position](#position)
    - [Orientation](#orientation)
    	- [Euler Angles](#euler-angles)

## Denavit-Hartenberg Diagram

This is a 6 DOF arm, with six linearly-connected revolute joints and a gripper attached to the 6th joint. Here is the Denavit-Hartenberg (DH) diagram of the arm:

![Denavit-Hartenberg diagram of the Kuka KR210 6-DoF arm][dh_diagram]




## Denavit-Hartenberg (Modified) Parameters

I obtained the following DH parameters from manipulator specifications (____ file). Note that these values relate to modified DH parameters.

| n | α(i-1) | a(i-1) | d(i) | θ(i) |
|:-:|:--:|:-----:|:------:|:---:|
| 1 | 0 | 0 | 0.75 | qi |
| 2 | - pi/2 | 0.35 | 0 | -pi/2 + q2 |
| 3 | 0 | 1.25 | 0 | q3 |
| 4 | -pi/2 | -0.054 | 1.5 | q4 |
| 5 | pi/2 | 0 | 0 | q5 |
| 6 | -pi/2 | 0 | 0 | q6 |
| EE | 0 | 0 | 0.303 | 0 |


The a (link length) and α (link twist) values for each of the links cannot change from pose to pose because they are specific to the arm. All movements of the arm are reflected using the d (link offset) values in the case of prismatic joints, and θ (joint angle) values in the case of revolute joints. Since this arm is a 6R arm (all revolute joints) only the θ parameters will be changing.

## Transformation Matrices

All of the joints frames have their own transformation matrix that describes their position and orientation relative to prior joint frames.

The transformation matrix can be calculated by substituting the DH parameters from the table above into this generic homogenous transformation matrix for each revolute joint:

```
T  = [[        cos(θ),       -sin(θ),       0,         a],
      [ sin(θ)*cos(α), cos(θ)*cos(α), -sin(α), -sin(α)*d],
      [ sin(θ)*sin(α), cos(θ)*sin(α),  cos(α),  cos(α)*d],
      [             0,             0,       0,         1]]
```

Using the generic transformation matrix above, here are the joint transformation matrices for the individual joints. Tx_y indicates the transformation to represent joint frame y in terms of joint frame x; in other words, the transformation needed to move the joint in frame x to the joint y:

```
T0_1:    [[ cos(θ1), -sin(θ1),  0,     0],
          [ sin(θ1),  cos(θ1),  0,     0],
          [       0,        0,  1,  0.75],
          [       0,        0,  0,     1]]
```

```
T1_2:    [[ sin(θ2),  cos(θ2),  0,  0.35],
          [       0,        0,  1,     0],
          [ cos(θ2), -sin(θ2),  0,     0],
          [       0,        0,  0,     1]]
```

```
T2_3:    [[ cos(θ3), -sin(θ3),  0,  1.25],
          [ sin(θ3),  cos(θ3),  0,     0],
          [       0,        0,  1,     0],
          [       0,        0,  0,     1]]
```

```
T3_4:    [[ cos(θ4), -sin(θ4),  0, -0.054],
          [       0,        0,  1,    1.5],
          [-sin(θ4), -cos(θ4),  0,      0],
          [       0,        0,  0,      1]]
```

```
T4_5:    [[ cos(θ5), -sin(θ5),  0,      0],
          [       0,        0, -1,      0],
          [ sin(θ5),  cos(θ5),  0,      0],
          [       0,        0,  0,      1]]
```

```
T5_6:    [[ cos(θ6), -sin(θ6),  0,      0],
          [       0,        0,  1,      0],
          [-sin(θ6), -cos(θ6),  0,      0],
          [       0,        0,  0,      1]]
```

And here is the transformation matrix for the gripper. Note that there is no rotation, but there is a translation in the Z direction (out forward).

```
T6_G: [[       1,        0,  0,      0],
          [       0,        1,  0,      0],
          [       0,        0,  1,  0.303],
          [       0,        0,  0,      1]]
```

## Homogeneous Transformation Matrix (Base to gripper)

To obtain one complete homogenous transformation matrix describing the displacements and rotations needed to get to the arm's' gripper with respect to the coordinate frame of the fixed base, we multiply all the joint transformation matrices together.

```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
= [[((sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*cos(theta5) + sin(theta5)*cos(theta1)*cos(theta2 + theta3))*cos(theta6) + (sin(theta1)*cos(theta4) - sin(theta4)*sin(theta2 + theta3)*cos(theta1))*sin(theta6), -((sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*cos(theta5) + sin(theta5)*cos(theta1)*cos(theta2 + theta3))*sin(theta6) + (sin(theta1)*cos(theta4) - sin(theta4)*sin(theta2 + theta3)*cos(theta1))*cos(theta6), -(sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*sin(theta5) + cos(theta1)*cos(theta5)*cos(theta2 + theta3), -0.303*(sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*sin(theta5) + (1.25*sin(theta2) - 0.054*sin(theta2 + theta3) + 1.5*cos(theta2 + theta3) + 0.35)*cos(theta1) + 0.303*cos(theta1)*cos(theta5)*cos(theta2 + theta3)],
 [((sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*cos(theta5) + sin(theta1)*sin(theta5)*cos(theta2 + theta3))*cos(theta6) - (sin(theta1)*sin(theta4)*sin(theta2 + theta3) + cos(theta1)*cos(theta4))*sin(theta6), -((sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*cos(theta5) + sin(theta1)*sin(theta5)*cos(theta2 + theta3))*sin(theta6) - (sin(theta1)*sin(theta4)*sin(theta2 + theta3) + cos(theta1)*cos(theta4))*cos(theta6), -(sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*sin(theta5) + sin(theta1)*cos(theta5)*cos(theta2 + theta3), -0.303*(sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*sin(theta5) + (1.25*sin(theta2) - 0.054*sin(theta2 + theta3) + 1.5*cos(theta2 + theta3) + 0.35)*sin(theta1) + 0.303*sin(theta1)*cos(theta5)*cos(theta2 + theta3)],
 [                                                                                           -(sin(theta5)*sin(theta2 + theta3) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*cos(theta6) - sin(theta4)*sin(theta6)*cos(theta2 + theta3),                                                                                              (sin(theta5)*sin(theta2 + theta3) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*sin(theta6) - sin(theta4)*cos(theta6)*cos(theta2 + theta3),                                                     -sin(theta5)*cos(theta4)*cos(theta2 + theta3) - sin(theta2 + theta3)*cos(theta5),                                                                   -0.303*sin(theta5)*cos(theta4)*cos(theta2 + theta3) - 0.303*sin(theta2 + theta3)*cos(theta5) - 1.5*sin(theta2 + theta3) + 1.25*cos(theta2) - 0.054*cos(theta2 + theta3) + 0.75],
 [                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                         0,                                                                                                                                    0,                                                                                                                                                                                                                                                1]]
```

As an example, let's calculate the homogenous transformation matrix with zero theta values -- i.e, neutral joint angles. So, substituting in zero for all the thetas in this equation yields:

```
T0_G_Neutral = [[ 1.0,   0,   0, 2.153],
           [   0, 1.0,   0,     0],
           [   0,   0, 1.0, 1.946],
           [   0,   0,   0,     1]]
```

Therefore, at its starting/neutral position, the gripper will lie at position [2.153 0 1.946].

## Inverse Kinematic Analysis

Inverse kinematics analysis is required to calculate the joint angles required to position the arm's gripper at the location required.

Given that the 6DoF arm has a spherical wrist (the last 3 joints -- J4, J5 and J6), we can decouple the IK into two parts ... the first is position analysis and next is orientation analysis.

Based on this approach, we utilize the first 3 joints to position the spherical wrist center (WC) where it needs to be (relative to the required position of the gripper arm), and we then orient the gripper arm into the desired orientation using the 3 joints of the spherical wrist. These final three joints of the spherical wrist are called the `wrist center` because the DH-convention frames of each of these joints intersects at one point.

Before proceeding, we must understand what information will be provided as the 'goal', since we have to work backwards from there. The information we receive as a "pose" instance is:
- Position of the gripper: px, py, pz
- Orientation of the gripper: roll, pitch, yaw

### Position

The position analysis is only concerned with the first three joints -- in order to position the WC (Wrist Center) of the spherical wrist. First, however, we must get the position of the WC, wrt to the position of the gripper.

![Inverse kinematic position calculation diagram][trigonometric_diagram]

Next, with the knowledge of the position of the WC, we can proceed to the IK analysis for the first 3 joint angles - θ1, θ2, θ3. The calculation is illustrated in this diagram.



Therefore, 

### Orientation

The orientation analysis requires us to determine the rotation matrix of joints 4, 5 and 6 (the spherical wrist) using the (roll, pitch, yaw) inputs as Tait-Bryan angles, and then calculate the requisite Euler angle equations wrt that rotation matrix. To get to that, some explanations and calculations are due...

Based on Tait-Bryan angles, the orientation of the gripper using (roll, pitch, yaw) would involve rotating first around the Z axis (yaw), followed by the Y axis (pitch) and then the X axis (roll).

The individual rotation matrices for rotations around an angle θ (which could be roll, pitch or yaw, respectively) for each of the X, Y and Z axes are:

```
Rx = [[				1,				 0,			      0],
	  [				0,        	cos(θ),       	-sin(θ)],
	  [				0,        	sin(θ),       	 cos(θ)]]


Ry = [[		   cos(θ),				 0,			  sin(θ)],
	  [				0,        		 1,       		  0],
	  [		  -sin(θ),        		 0,       	  cos(θ)]]

Rz = [[			cos(θ),		   -sin(θ),			      0],
	  [			sin(θ),        	cos(θ),       		  0],
	  [				0,        		 0,       	 	  1]]
```

Therefore, using (roll, pitch, yaw) as Tait-Bryan angles, the combined transformation matrix for the gripper, with respect to the base frame, and inputs (px, py, pz, roll, pitch, yaw), is given by:
```
T_EE' = Rx(yaw) * Ry(pitch) * Rz(yaw)
		= Matrix[
	  		[cos(pitch)*cos(yaw),       	sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll),  	sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw),  	px],
	  		[sin(yaw)*cos(pitch),			sin(pitch)*sin(roll)*sin(yaw) + cos(yaw)*cos(roll), 		sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw), 		py],
	  		[		 -sin(pitch),				sin(roll)*cos(pitch),									cos(pitch)*cos(roll),								pz],
      		[             	0,             												0,       												     0,     1]]
```

A caveat in this case, is that to get the correct transformation of the gripper wrt the base frame, we must also translate the the gripper's frame which is in the URDF convention, into the DF-convention, prior to using the transformation matrix in our calculations. This correction requires correcting the rotation matrix (T_EE') above to include a rotation of the gripper's coordinate frame around the Z axis by 180 degrees, first, and then around the Y axis by -90 degrees:

```
T_EE = T_EE' * Rz(180) * Ry(-90)
		= Matrix[
	  		[cos(pitch)*coss(roll)*cos(yaw) + sin(roll)*sin(yaw),  	-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll),  	cos(pitch)*cos(yaw),  	px],
	  		[sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw), 	-sin(pitch)*sin(roll)*sin(yaw) - cos(yaw)*cos(roll), 	sin(yaw)*cos(pitch), 	py],
	  		[							   cos(pitch)*cos(roll),								  -sin(roll)*cos(pitch),				-sin(pitch),		pz],
      		[             									  0,             										  0,						  0,     1]]
```


First for the rotational matrix for the spherical wrist (R3_6). An important step in this determination is to correct for the 


The rotation matrix of the spherical wrist (R3_6) is calculated as:

```python
R3_6 = R0_3.T * Rrpy * R_corr
```

Where:
- `R0_3.T` is the transposition of the rotation matrix from joints 1, 2 and 3, requiring the angles derived in the inverse kinematic position calculations
- `Rrpy` is the rotation matrix of the gripper's current roll, pitch and yaw
- `R_corr` is the rotation matrix of the gripper correction matrix that rotates the gripper around the Z axis by 180 degrees and around the Y axis by -90 degrees

With this rotation matrix, it is possible to derive the Euler angles.

#### Euler Angles

[tf transformations](http://www.lfd.uci.edu/~gohlke/code/transformations.py.html)
See function called `euler_from_matrix` that takes in a numpy rotation matrix and Euler axis sequence, and returns the three Euler angles (alpha, beta and gamma).

The rotation matrix above uses the Euler definition of `XYZ`, which is a Tait-Bryan angle combination. With the alpha, beta and gamma angles, I mapped them to theta 4, theta 5 and theta 6, respectively.

However, theta 4 and theta 5 required these additional calculations:

```python
theta4 = np.pi/2 + theta4
theta5 = np.pi/2 - theta5
```
