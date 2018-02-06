---
title: "Kinematics Pick & Place Report"
author: [Piotr Sókolski]
date: 2018-02-05
...

[//]: # (Image References)

[links]: ../misc_images/links.jpeg
[t1-3]: ../misc_images/t1-3.jpeg
[playground]: ../misc_images/playground.png
[eq3]: ../misc_images/eq3.png {width=200px}

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
#### Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![Link and origin frames' assignments][links]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
$T^0_1$ | 0 | 0 | 0.75 | q1
$T^1_2$ | - pi/2 | 0.35 | 0 | -pi/2 + q2
$T^2_3$ | 0 | 1.25 | 0 | q3
$T^3_4$ |  - pi/2 | -0.054 | 1.5 | q4
$T^4_5$ | pi/2 | 0 | 0 | q5
$T^5_6$ | -pi/2 | 0 | 0 | q6
$T^6_{EE}$ | 0 | 0 | 0.303 | 0

$$
\begin{aligned}
T^0_1 &= \left[\begin{matrix}\cos{\left (q_{1} \right )} & - \sin{\left (q_{1} \right )} & 0 & 0\\\sin{\left (q_{1} \right )} & \cos{\left (q_{1} \right )} & 0 & 0\\0 & 0 & 1 & 0.75\\0 & 0 & 0 & 1\end{matrix}\right]
T^1_2 = &\left[\begin{matrix}\sin{\left (q_{2} \right )} & \cos{\left (q_{2} \right )} & 0 & 0.35\\0 & 0 & 1 & 0\\\cos{\left (q_{2} \right )} & - \sin{\left (q_{2} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]\\
T^2_3 &= \left[\begin{matrix}\cos{\left (q_{3} \right )} & - \sin{\left (q_{3} \right )} & 0 & 1.25\\\sin{\left (q_{3} \right )} & \cos{\left (q_{3} \right )} & 0 & 0\\0 & 0 & 1 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
T^3_4 = &\left[\begin{matrix}\cos{\left (q_{4} \right )} & - \sin{\left (q_{4} \right )} & 0 & -0.054\\0 & 0 & 1 & 1.5\\- \sin{\left (q_{4} \right )} & - \cos{\left (q_{4} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]\\
T^4_5 &= \left[\begin{matrix}\cos{\left (q_{5} \right )} & - \sin{\left (q_{5} \right )} & 0 & 0\\0 & 0 & -1 & 0\\\sin{\left (q_{5} \right )} & \cos{\left (q_{5} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
T^5_6 = &\left[\begin{matrix}\cos{\left (q_{6} \right )} & - \sin{\left (q_{6} \right )} & 0 & 0\\0 & 0 & 1 & 0\\- \sin{\left (q_{6} \right )} & - \cos{\left (q_{6} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]\\
T^6_{EE} &= \left[\begin{matrix}1 & 0 & 0 & 0\\0 & 1 & 0 & 0\\0 & 0 & 1 & 0.303\\0 & 0 & 0 & 1\end{matrix}\right]
\end{aligned}
$$

The homogeneous transform matrix from base link can be constructed from the input data:

$$
\resizebox{.9\hsize}{!}{$
R_T = R_z \cdot R_y \cdot R_x \cdot R_{gripper} = \\
\left[\begin{matrix}\sin{\left (r_{x} \right )} \sin{\left (r_{z} \right )} + \sin{\left (r_{y} \right )} \cos{\left (r_{x} \right )} \cos{\left (r_{z} \right )} & - \sin{\left (r_{x} \right )} \sin{\left (r_{y} \right )} \cos{\left (r_{z} \right )} + \sin{\left (r_{z} \right )} \cos{\left (r_{x} \right )} & \cos{\left (r_{y} \right )} \cos{\left (r_{z} \right )}\\- \sin{\left (r_{x} \right )} \cos{\left (r_{z} \right )} + \sin{\left (r_{y} \right )} \sin{\left (r_{z} \right )} \cos{\left (r_{x} \right )} & - \sin{\left (r_{x} \right )} \sin{\left (r_{y} \right )} \sin{\left (r_{z} \right )} - \cos{\left (r_{x} \right )} \cos{\left (r_{z} \right )} & \sin{\left (r_{z} \right )} \cos{\left (r_{y} \right )}\\\cos{\left (r_{x} \right )} \cos{\left (r_{y} \right )} & - \sin{\left (r_{x} \right )} \cos{\left (r_{y} \right )} & - \sin{\left (r_{y} \right )}\end{matrix}\right]$}
$$

![Homogenous Transform Matrix][eq3]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![Solution for $\theta_{1-3}$ \label{t13}][t1-3]

$\theta_{4-6}$ is computed by solving $R^3_{EE} = (R^0_3)^{-1} \cdot R^0_{EE} = (R^0_3)^T \cdot R^0_{EE}$.
Since

$$
\resizebox{.9\hsize}{!}{$
R^3_{EE} = \left[\begin{matrix}- \sin{\left (q_{4} \right )} \sin{\left (q_{6} \right )} + \cos{\left (q_{4} \right )} \cos{\left (q_{5} \right )} \cos{\left (q_{6} \right )} & - \sin{\left (q_{4} \right )} \cos{\left (q_{6} \right )} - \sin{\left (q_{6} \right )} \cos{\left (q_{4} \right )} \cos{\left (q_{5} \right )} & - \sin{\left (q_{5} \right )} \cos{\left (q_{4} \right )}\\\sin{\left (q_{5} \right )} \cos{\left (q_{6} \right )} & - \sin{\left (q_{5} \right )} \sin{\left (q_{6} \right )} & \cos{\left (q_{5} \right )}\\- \sin{\left (q_{4} \right )} \cos{\left (q_{5} \right )} \cos{\left (q_{6} \right )} - \sin{\left (q_{6} \right )} \cos{\left (q_{4} \right )} & \sin{\left (q_{4} \right )} \sin{\left (q_{6} \right )} \cos{\left (q_{5} \right )} - \cos{\left (q_{4} \right )} \cos{\left (q_{6} \right )} & \sin{\left (q_{4} \right )} \sin{\left (q_{5} \right )}\end{matrix}\right]$}
$$

and $(R^0_3)^T \cdot R^0_{EE}$ can be computed from input and $\theta_{1-3}$ we have:

```python
for t5_flip in [1, 0]:
  # Theta5 has an alternative solution at -Theta5
  theta5 = (1 - 2*t5_flip) * atan2(sinq5, cosq5)

  sgn = sin(theta5)
  sinq4 = R3_EE[2, 2]/sgn
  cosq4 = -R3_EE[0, 2]/sgn
  theta4 = atan2(sinq4, cosq4)

  sinq6 = -R3_EE[1, 1]/sgn
  cosq6 = R3_EE[1, 0]/sgn
  theta6 = atan2(sinq6, cosq6)
```

##### Mutliple solutions

Both $\sin{\beta}$ from figure \ref{t13} and $\theta_5$ can have more than one
solution.
For an alternative solution to $\theta_5$, the solution at $-\theta_5$ effectively forces a 180 dg offset to $\theta_4$ and $\theta_6$, and the arm ends up in a "flipped wrist" pose.
Additionally each joint can be rotated by $\pm360^{\circ}$, as long as it meets the joint specifications.
In my program I consider all possible solutions and choose the most cost-effective configuration with regard to transition time between joint poses.
The algorithm is explained in detail in the next section.

### Project Implementation

I've implemented the solution in `inverse_kinematics.py` in the same directory.
For dealing with multiple solutions (regarding the two possible solutions for $\theta_3$, $\theta_5$ or rotation by $\pm360^{\circ}$) I compute them all (`SolveIK`, `SolveIKLimits`).
Then I discard the solutions that are impossible because of joint constraints specified in the urdf file (`SolveIKLimits`).
Later I compute the cost of each solution by taking the maximum time that joint would travel from a previous position to a new position using joint velocities specified in the urdf model.
I select the solution with the smallest cost.

Without the cost computation, if any solution is selected, the arm would often just rotate the last few joints in place to an alternative configuration that yields the same EE position. I've found that using cost and computing maximum instead of a sum in the cost computation improves the solution greatly, since the joints can operate in parallel.
Even though the solution where the third joint is closer to the floor would not be very useful in the given world configuration, I've found that the cost model would be able to efficiently use it from time to time.

By considering the alternative solutions the robot no longer performs rotations in place, although some unnecessary rotations are still part of the motion plan.
This should be optimized in the motion planner or in postprocessing.

My code always assumes there is a solution, I've not worked the cases where the EE position would be out of range or the computations would lead to a division by zero.
Especially $\theta_5 = 0$ leads to an infinite number of solutions for $\theta_4$ and $\theta_6$.

#### Results

I've evaluated the motion planner commands and IK-FK output during simulation.
The error between desired and computed EE position is small, less than 0.00001 on
average.
The robot is able to follow the trajectory perfectly.
See the last section of `computations.ipynb` for code.

The robot is usually able to pick up the can, I believe the only times it fails is due to a time lag while grasping.

The simulation would stutter and follow the trajectory in small increments, even though it works smoothly in demo mode.
This can be seen on [a video](https://www.youtube.com/watch?v=XAMD_Qh0R60).
I'm not sure what the reason for that would be.

#### Sidenotes

I've used a quaternion to a rotation matrix implementation I've found online to be
able to work on the solution on a computer without ROS installed.

I've found Jupyter notebook's interactive widgets useful for testing forward
kinematics (see 'computations.ipynb' Forward Kinematics Playground).

![playground][playground]
