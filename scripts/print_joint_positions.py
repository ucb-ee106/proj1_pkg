#!/usr/bin/env python
import rospy
import sys
from utils.utils import *
import baxter_interface
from path_planner import PathPlanner

"""
This script starts up a node that prints the Baxter's joint angles in real time in the form
of a copy-pasteable python list. Use this to find the joint angles in the right order to be 
consumed by other parts of the codebase.

Takes as command line argument "left" or "right" to specify an arm.

Example:

rosrun proj1_pkg print_joint_positions.py left
rosrun proj1_pkg print_joint_positions.py right

"""

rospy.init_node("joint_position_printer")

limb = baxter_interface.Limb(sys.argv[1])

while not rospy.is_shutdown():
	joint_pos = repr(list(get_joint_positions(limb)))
	print(joint_pos)
