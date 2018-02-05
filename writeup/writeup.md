---
title: "Kinematics Pick & Place Report"
author: [Piotr Sókolski]
date: 2018-02-05
...

[//]: # (Image References)

[links]: ../misc_images/links.jpeg
[t1-3]: ../misc_images/t1-3.jpeg
[playground]: ../misc_images/playground.png

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
$T^6_EE$ | 0 | 0 | 0.303 | 0

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
cosq5 = R3_EE[1, 2]
for t5sgn in [1, -1]:
  sinq5 = t5sgn * sqrt(1 - cosq5 ** 2)
  theta5 = atan2(sinq5, cosq5)

  sinq4 = t5sgn * R3_EE[2, 2]
  cosq4 = -t5sgn * R3_EE[0, 2]
  theta4 = atan2(sinq4, cosq4)

  sinq6 = -t5sgn * R3_EE[1, 1]
  cosq6 = t5sgn * R3_EE[1, 0]
  theta6 = atan2(sinq6, cosq6)
```

##### Mutliple solutions

Both $\sin{\beta}$ from figure \ref{t13} and $\sin{\theta_5}$ can have more than one
solution. Additionally each joint can be rotated by $\pm360^{\circ}$, as long as it
meets the joint specifications. In my program I consider all possible solutions and
choose the most cost-effective configuration with regard to transtion time between
joint poses. The algorithm is explained in detail in the next section.

### Project Implementation

I've implemented the solution in `inverse_kinematics.py` in the same directory. For
dealing with multiple solutions (regarding the two possible solutions for $\theta_3$,
$\theta_5$ or rotation by $\pm360^{\circ}$) I compute them all (`SolveIK`). Then I
discard the solutions that are impossible because of joint constraints specified in the urdf file (`SolveIKLimits`).
Later I compute the cost of each solution by taking the maximum
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

#### Sidenotes

I've used a quaternion to a rotation matrix implementation I've found online to be
able to work on the solution on a computer without ROS installed.

I've found Jupyter notebook's interactive widgets useful for testing forward
kinematics (see 'computations.ipynb' Forward Kinematics Playground).

![playground][playground]

The error between desired and computed EE position is small, less than 0.00001 on
average. See the last section of `computations.ipynb` for code.
