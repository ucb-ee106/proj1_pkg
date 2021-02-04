#!/usr/bin/env python
import rospy
import sys
from utils.utils import *
import baxter_interface
from path_planner import PathPlanner

"""
Puts the arms of Baxter in a state where the elbow is straightened out. Run it with either
"low_stretch" or "high stretch" options specified.

Examples:

rosrun proj1_pkg relax_arms.py low_stretch
rosrun proj1_pkg relax_arms.py high_stretch

You can also use this script as a starting point to come up with other convenient arm positions.
Just use the joint_position_keyboard.py script from baxter_examples to manipulate the joints directly.

"""

rospy.init_node("relax_arms")

if sys.argv[1] == 'low_stretch':
	ql = [-0.7995570902191504, -1.0230128644807106, 0, 1.9412961426428978, 0, 0.6088981529125705, 0]
	qr = [0.7995570902191504, -1.0230128644807106, 0, 1.9412961426428978, 0, 0.6088981529125705, 0]

elif sys.argv[1] == 'high_stretch':
	ql = [-0.7995570902191504, -0.3583801252107035, 0, 0.8317392022589729, 0, 1.072678159715874, 0]
	qr = [0.7995570902191504, -0.3583801252107035, 0, 0.8317392022589729, 0, 1.072678159715874, 0]
else:
	print("Please run this script with either 'low_stretch' or 'high_stretch' specified as a command line argument.")
	sys.exit()

planner = PathPlanner('left_arm')
success, plan, time_taken, error_code = planner.plan_to_joint_pos(ql)
planner.execute_plan(plan)

planner = PathPlanner('right_arm')
success, plan, time_taken, error_code = planner.plan_to_joint_pos(qr)
planner.execute_plan(plan)
