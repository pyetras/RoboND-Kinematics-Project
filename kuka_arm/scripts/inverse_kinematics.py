from sympy import symbols, sin, cos, pi, pprint, atan2, sqrt
from sympy.matrices import Matrix, Transpose
from functools import reduce
import numpy as np

def rot_x(q):
  return Matrix([[1, 0, 0],
                 [0, cos(q), -sin(q)],
                 [0, sin(q), cos(q)]])

def rot_y(q):
  return Matrix([[cos(q), 0, sin(q)],
                 [0, 1, 0],
                 [-sin(q), 0, cos(q)]])

def rot_z(q):
  return Matrix([[cos(q), -sin(q), 0],
                 [sin(q), cos(q), 0],
                 [0, 0, 1]])

def quat2mat(quat):
    ''' Symbolic conversion from quaternion to rotation matrix
    For a unit quaternion

    From: http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    Originally from: https://github.com/matthew-brett/transforms3d
    '''
    x, y, z, w = quat
    return Matrix([
            [1 - 2*y*y-2*z*z, 2*x*y - 2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])

def _CreateTransform(R, T):
  return R.row_join(T).col_join(Matrix([[0,0,0,1]]))

def DHTransform(alpha, a, q, d):
  return Matrix([
    [cos(q),            -sin(q),           0,           a],
    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
    [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d ],
    [0,                 0,                 0,           1            ]])

# Constants.

q1, q2, q3, q4, q5, q6 = symbols('q1:7')
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0]
a = [0, 0.35, 1.25, -0.054, 0, 0, 0]
d1, d2, d3, d4, d5, d6, d_ee = [0.75, 0, 0, 1.5, 0, 0, 0.303]
R_gripper = rot_z(pi) * rot_y(-pi/2)
d_star = sqrt(0.054**2 + 1.5**2)
d_z = d1
d_x = a[1]
velocities = np.array([123., 115., 112., 179., 172., 219.])
limits = [
  [-185, 185],
  [-45, 85],
  [-210, 155 - 90],
  [-350, 350],
  [-125, 125],
  [-350, 350]]

def DHMatrices():
  T0_1 = DHTransform(alpha[0], a[0], q1, d1)
  T1_2 = DHTransform(alpha[1], a[1], q2 - pi/2, d2)
  T2_3 = DHTransform(alpha[2], a[2], q3, d3)
  T3_4 = DHTransform(alpha[3], a[3], q4, d4)
  T4_5 = DHTransform(alpha[4], a[4], q5, d5)
  T5_6 = DHTransform(alpha[5], a[5], q6, d6)
  T6_EE = DHTransform(alpha[6], a[6], 0, d_ee)

  return [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_EE]

R0_3 = reduce(lambda acc, T: acc[0:3,0:3]*T[0:3,0:3], DHMatrices()[:3])

def GetCost(prev_thetas, thetas):
  cost = np.sum(np.abs(np.array(prev_thetas) - np.array(thetas)) / velocities)
  return cost

def SolveIKCheapest(prev_thetas, *args, **kwargs):
  return min([(GetCost(prev_thetas, thetas), thetas)
      for thetas in SolveIKLimits(*args, **kwargs)])[1]

def SolveIKLimits(*args, **kwargs):
  dtr = np.pi / 180.0

  def SolveRec(thetas, ix, ans):
    if ix == 6:
      yield ans[:]
      return

    theta = thetas[ix]
    left, right = limits[ix]
    for shift in [0, -2*np.pi, 2*np.pi]:
      if left*dtr <= (theta + shift) <= right*dtr:
        ans.append(theta + shift)
        for output in SolveRec(thetas, ix + 1, ans):
          yield output
        ans.pop()

  for thetas in SolveIK(*args, **kwargs):
    for ans in SolveRec(thetas, 0, []):
      yield ans

def SolveFK(thetas):
  return reduce(lambda acc, T: acc * T, DHMatrices())[0:3,3]\
      .evalf(subs = dict(zip([q1,q2,q3,q4,q5,q6], thetas)))

def SolveFKwc(thetas):
  return reduce(lambda acc, T: acc * T, DHMatrices()[:4])[0:3,3]\
        .evalf(subs = dict(zip([q1,q2,q3,q4], thetas[:3] + [0])))

def SolveIK(ee_pos_vals, quaternion_vals, debug = False):
  global R0_3

  ee_pos = Matrix(ee_pos_vals)
  T = _CreateTransform(quat2mat(quaternion_vals), ee_pos)

  R0_EE = T[0:3,0:3] * R_gripper
  v_EE = T[0:3,3]
  wc_pos = v_EE - R0_EE*Matrix([0,0,d_ee])

  if debug:
    print("wc_pos")
    pprint(wc_pos)

  # Theta 1-3

  xc, yc, zc = wc_pos
  qq1 = atan2(yc, xc)

  # q3
  alp = atan2(-0.054, 1.5)
  s = zc - d_z
  r = sqrt(xc**2 + yc**2) - d_x
  cosbet = (r**2 + s**2 - a[2]**2 - d_star**2)/(2*a[2]*d_star)
  for q3sign in [1]: #, -1]:
    sinbet = q3sign * sqrt(1 - cosbet**2)

    qq3 = -(pi/2 - (atan2(sinbet, cosbet) + alp))
    # q2
    qq2 = pi/2 - (atan2(s, r) + atan2(d_star*sinbet, a[2] + d_star*cosbet))

    # Theta 4-6
    R0_3_eval = R0_3.evalf(subs = {q1: qq1, q2: qq2, q3: qq3})

    R3_EE = Transpose(R0_3_eval) * R0_EE
    sinq5 = sqrt(R3_EE[0,2]**2 + R3_EE[2, 2]**2)
    cosq5 = R3_EE[1,2]
    qq5 = atan2(sinq5, cosq5)

    sinq4 = R3_EE[2,2]
    cosq4 = -R3_EE[0,2]
    qq4 = atan2(sinq4, cosq4)

    sinq6 = -R3_EE[1, 1]
    cosq6 = R3_EE[1, 0]
    qq6 = atan2(sinq6, cosq6)

    thetas = [qq1,qq2,qq3,qq4,qq5,qq6]

    if debug:
      print("computed wc_pos")
      pprint(SolveFKwc(thetas))

      pprint("P0_EE")
      pprint(T[0:3,3])

      print("Computed P0_EE")
      pprint(SolveFK(thetas))

    yield Matrix(thetas).evalf()
