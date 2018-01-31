from sympy import symbols, sin, cos, pi, pprint, simplify, atan2, sqrt
from sympy.matrices import Matrix, Transpose
import numpy as np
from functools import reduce

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])

    return R_x

def rot_y(q):
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])

    return R_y

def rot_z(q):
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])

    return R_z

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

def joinz(R, T):
  return R.row_join(T).col_join(Matrix([[0,0,0,1]]))

def DHTransform(alpha, a, q, d):
  return Matrix([
    [cos(q),            -sin(q),           0,           a],
    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
    [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d ],
    [0,                 0,                 0,           1            ]])

q1, q2, q3, q4, q5, q6 = symbols('q1:7')
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0]
a = [0, 0.35, 1.25, -0.054, 0, 0, 0]
d1, d2, d3, d4, d5, d6, d_ee = [0.75, 0, 0, 1.5, 0, 0, 0.303]
R_gripper = rot_z(pi) * rot_y(-pi/2)

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

def SolveIK(ee_pos_vals, quaternion_vals):
  global R0_3

  ee_pos = Matrix(ee_pos_vals)
  T = joinz(quat2mat(quaternion_vals), ee_pos)

  R0_EE = T[0:3,0:3] * R_gripper
  v_EE = T[0:3,3]
  wc_pos = simplify(v_EE - R0_EE*Matrix([0,0,d_ee]))

  print("wc_pos")
  pprint(wc_pos)

  xc, yc, zc = wc_pos
  qq1 = atan2(yc, xc)

  # q3
  dd = sqrt(0.054**2 + 1.5**2)
  dz = d1
  dx = a[1]
  s = zc - dz
  r = sqrt(xc**2 + yc**2) - dx
  cosbet = (r**2 + s**2 - a[2]**2 - dd**2)/(2*a[2]*dd)
  sinbet = sqrt(1 - cosbet**2)

  qq3 = -atan2(cosbet, sinbet) + atan2(-0.054, 1.5)

  # q2
  qq2 = -(atan2(s, r) - atan2(a[2] + d4*cosbet, d4*sinbet))

  R0_3 = R0_3.evalf(subs = {q1: qq1, q2: qq2, q3: qq3})

  R3_EE = Transpose(R0_3) * R0_EE * R_gripper

  sinq5 = sqrt(R3_EE[1,1]**2 + R3_EE[1, 2]**2)
  cosq5 = R3_EE[1,0]
  qq5 = atan2(sinq5, cosq5)

  cosq4 = R3_EE[0,0]*-1
  sinq4 = R3_EE[2,0]
  qq4 = atan2(sinq4, cosq4)

  cosq6 = R3_EE[1, 2]
  sinq6 = R3_EE[1, 1]
  qq6 = atan2(sinq6, cosq6)

  print("computed wc_pos")
  pprint(reduce(lambda acc, T: acc * T, DHMatrices()[:4])
    .subs([(q1, qq1), (q2, qq2), (q3, qq3), (q4, 0)])[0:3,3])

  pprint("R0_EE")
  pprint(T)

  pprint("computed R0_EE")
  pprint(reduce(lambda acc, T: acc * T, DHMatrices())
    .subs([(q1, qq1), (q2, qq2), (q3, qq3), (q4, qq4), (q5, qq5), (q6, qq6)]))

  return Matrix([qq1,qq2,qq3,qq4,qq5,qq6]).evalf()
