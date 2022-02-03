#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("base", "right_gripper_tip")

seed_state = [0.0] * ik_solver.number_of_joints

ik_sol = ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,
                0.0, 0.0, 0.0, 1.0,
                0.01, 0.01, 0.01,  # X, Y, Z bounds
                0.1, 0.1, 0.1)  # Rotation X, Y, Z bounds
print ik_sol