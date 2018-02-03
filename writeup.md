## Project: Kinematics Pick & Place

[//]: # (Image References)

[links]: ./misc_images/links.jpeg
[t1-3]: ./misc_images/t1-3.jpeg
[matrix]: ./misc_images/matrix.jpeg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![links][links]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta 1-3

![theta 1-3][t1-3]

Theta 4-6 is solved by solving R^3_E = (R^0_3)^T * R^0_EE, where R^3_EE is:

![matrix][matrix]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

I've implemented the solution in `inverse_kinematics.py` in the same directory. For
dealing with multiple solutions (regarding the two possible solutions for theta3 or
rotation by +/- 360 degrees) I compute them all (`SolveIK`). Then I discard the
solutions that are impossible because of joint constraints specified in the urdf file
(`SolveIKLimits`). Later I compute the cost of each solution by taking the maximum
time that joint would travel from a previous position to a new position using joint
velocities specified in the urdf model. I select the solution with the smallest cost.

Without the cost computation, if any solution is selected, the arm would often just
rotate the last few joints in place to an alternative configuration that yields the
same EE poision. I've found that using cost and computing maximum instead of a sum in
the cost computation improves the solution greatly, since the joints can operate in
parallel. Even though -sin(beta) would not be very useful in the given world configuration,
I've found that the cost model would be able to efficiently use it from time to time.

This is not perfect, as the robot would still perform some unnecessary rotations in place,
perhaps a model involving more configurations than just the current configuration or
a heuristic ranking the solutions would perform better.

My code always assumes there is a solution, I've not worked the cases where the EE pos
would be out of range or the computations would lead to a division by zero.

I've used a quaternion to a rotation matrix implementation I've found online to be
able to work on the solution on a computer without ROS installed.
