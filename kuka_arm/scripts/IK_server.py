#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import inverse_kinematics as ik
from datetime import datetime
import numpy as np

prev_thetas = [0]*6

def handle_calculate_IK(req):
    queries = []

    global prev_thetas

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            quaternion = [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w]

            # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

            thetas = ik.SolveIKCheapest(prev_thetas, [px, py, pz], quaternion)
            queries.append([px, py, pz] + quaternion + prev_thetas + thetas)

            # if IK_error(thetas, [px, py, pz]) > 0.1:
            #     print([[px, py, pz], quaternion])
            #     print([ik.SolveFK(thetas), thetas, prev_thetas])
            prev_thetas = thetas

            joint_trajectory_point.positions = thetas
            joint_trajectory_list.append(joint_trajectory_point)

        # timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
        # np.savetxt(
        #     '/home/robond/catkin_ws/src/RoboND-Kinematics-Project/data/queries_%s.csv'%timestamp,
        #     np.asarray(queries), delimiter=',')
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)

def IK_error(thetas, test_pos):
    your_ee = ik.SolveFK(thetas)
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_pos[0])
        ee_y_e = abs(your_ee[1]-test_pos[1])
        ee_z_e = abs(your_ee[2]-test_pos[2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        rospy.loginfo("End effector error for x position is: %04.8f" % ee_x_e)
        rospy.loginfo("End effector error for y position is: %04.8f" % ee_y_e)
        rospy.loginfo("End effector error for z position is: %04.8f" % ee_z_e)
        rospy.loginfo("Overall end effector offset is: %04.8f units \n" % ee_offset)
        return ee_offset


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
