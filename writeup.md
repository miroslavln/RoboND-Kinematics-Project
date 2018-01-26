## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/drawing.jpg
[image2]: ./misc_images/graphic.jpg
[image3]: ./misc_images/formula1.jpg
[image4]: ./misc_images/urdf_parameters.jpg
[image5]: ./misc_images/dh-transform-matrix.png
[image6]: ./misc_images/inverse_kinematics1.png
[image7]: ./misc_images/formula2.jpg
[image8]: ./misc_images/gazeebo.png
[image9]: ./misc_images/matrices.png
[image10]: ./misc_images/R3_6.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

First we need to separate the robot into links and joins. Here is what it looks like.
![alt text][image1]

Now we need to find the lengths of the individual joints. We will use the kr210.urfd.xacro file to get the values. Here is an example of the information provided.  
```xml
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
```
The resulting table is as shown on the following image. 

![alt text][image4]


This is the resulting DH table

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

For an example the di for transformation 0->1 is calculated summing the z value for joint_1 and joint_2 (0.33 + 0.42) resulting in d1 of 0.75

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
![alt text][image5]
```python
def create_transformation_matrix(alpha, a, d, q):
    return Matrix([[              cos(q),            -sin(q),              0,                a],
                         [   sin(q)*cos(alpha),  cos(q)*cos(alpha),    -sin(alpha),    -sin(alpha)*d],
                         [   sin(q)*sin(alpha),  cos(q)*sin(alpha),     cos(alpha),     cos(alpha)*d],
                         [                   0,                  0,              0,                1]])
                         
T0_1 = create_transformation_matrix(alpha0, a0, d1, q1).subs(DH)
T1_2 = create_transformation_matrix(alpha1, a1, d2, q2).subs(DH)
T2_3 = create_transformation_matrix(alpha2, a2, d3, q3).subs(DH)
T3_4 = create_transformation_matrix(alpha3, a3, d4, q4).subs(DH)
T4_5 = create_transformation_matrix(alpha4, a4, d5, q5).subs(DH)
T5_6 = create_transformation_matrix(alpha5, a5, d6, q6).subs(DH)
T6_G = create_transformation_matrix(alpha6, a6, d7, q7).subs(DH)

```
The transformation from base link to gripper is done by multiplying all the transition tables. 

![alt text][image9]
```python
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
Resulting matrix
```python
T0_G = [[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),-0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2)- 0.054*cos(q2 + q3) + 0.75],
[0,0,0,1]]
```

```python
R_x = Matrix([[ 1,              0,          0],
			  [ 0,        cos(roll), -sin(roll)],
			  [ 0,        sin(roll),  cos(roll)]])


R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
			  [          0,        1,           0],
			  [-sin(pitch),        0,  cos(pitch)]])


R_z = Matrix([[ cos(yaw), -sin(yaw), 0],
			  [ sin(yaw),  cos(yaw), 0],
			  [ 0,              0,     1]])

Rrpy = R_z * R_y * R_x
```
We also need to add a correction for the gripper orientation

```python
R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi / 2)
Rrpy = Rrpy * R_corr
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

To solve the inverse kinematics problem we use a wrist center to simplify the problem. 
```
WC = Matrix([px, py, pz]) - d7 * Rrpy * Matrix([1,0,0])
```
Here the px,py, pz are the desired position of the gripper, d7 is the distance to joint 5 which is designated as the wrist center. 
theta1 is easily calculated from the top view projection
![alt text][image6]
```python
theta1 = atan2(wy, wx)
```
![alt text][image2]
From the graphic above we get the following formulas
![alt text][image3]
and finally we can derive theta2 and theta3
![alt text][image7]
Using the above triangle we can solve for theta2, and theta3 by using the cosine law. 
```python
r = sqrt(wx ** 2 + wy ** 2) - 0.35

A = 1.501 #sqrt(a3**2 + d4**2)
B = sqrt(r ** 2 + (wz - 0.75) ** 2)
C = 1.25 #a2

alpha = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
beta = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))

theta2 = pi / 2 - alpha - atan2(wz - 0.75, r)
theta3 = pi / 2 - beta + 0.036 
```
After we have the first 3 angles then we can calculate the rest using matrix inversion. 
```python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.T * Rrpy
```
![alt text][image7]
```python
theta5 = atan2(sqrt(r13**2 + r33**2), r23)
if sin(theta5 ) < 0:
    theta4  = atan2(-r33, r13)
    theta6 = atan2(r22, -r21)
else:
    theta4 = atan2(r33, -r13)
    theta6 = atan2(-r22, r21)
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The implementation is using the sympy library. I initially used the inverse of the R0_3 to calculate the inverse kinematics 
but that was very slow so I changed to use the transpose instead which improved performance significantly. I also had problems with the gripper 
which was not waiting enough time to grab the object but after an adjustment to the trajectory_sampler.cpp it started working. 
Here is an image after a few executions.
![alt text][image8]




