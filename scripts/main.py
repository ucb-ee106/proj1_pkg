#!/usr/bin/env python
"""
Starter script for lab1. 
Author: Chris Correa
"""
import sys
import argparse
import time
import numpy as np

from paths.paths import LinearPath, CircularPath
from utils.utils import *
from path_planner import PathPlanner

from trac_ik_python.trac_ik import IK

import rospy
import tf
import tf2_ros
import baxter_interface
import intera_interface
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState
from baxter_pykdl import baxter_kinematics

def get_trajectory(limb, kin, ik_solver, planner, args):
    """
    PROJECT 1 PART A

    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = args.num_way
    controller_name = args.controller_name
    task = args.task

    # Instantiate the MotionPath subclasses that you implemented in paths.py
    if task == 'line':
        path = None
        return path.to_robot_trajectory(num_way, controller_name!='workspace')

    elif task == 'circle':
        path = None
        return path.to_robot_trajectory(num_way, controller_name!='workspace')

    elif task == 'arbitrary':
        # Use moveit to create a plan from the robot's current configuration to an arbitrary
        # target configuration.

        target_position = None
        target_orientation = None # orientation specified as quaternion.

        pose = create_pose_stamped_from_pos_quat(
            target_position,
            target_orientation,
            'base'
        )
        success, plan, time_taken, error_code = planner.plan_to_pose(pose)
        jointspace_path = plan

        if args.controller_name == 'jointspace':
            path = jointspace_path
        elif args.controller_name == 'workspace':
            # PROJECT 1 PART B
            # To be able to use our workspace controller to follow MoveIt generate paths,
            # we need to convert them from jointspace trajectories into workspace trajectories.
            path = convert_jointspace_to_workspace_trajectory(jointspace_path, limb, kin)
            sys.exit()
        return path
    else:
        raise ValueError('task {} not recognized'.format(task))

def get_controller(controller_name, limb, kin):
    """
    PROJECT 1 PART B

    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'workspace':
        # YOUR CODE HERE
        Kp = None
        Kv = None
        controller = WorkspaceVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'jointspace':
        # YOUR CODE HERE
        Kp = None
        Kv = None
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'torque':
        # YOUR CODE HERE
        Kp = None
        Kv = None
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    elif controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

def main():
    """
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is.

    NOTE: If running with the --moveit flag, it makes no sense
    to also choose your controller to be workspace, since moveit
    cannot accept workspace trajectories. This script simply ignores
    the controller selection if you specify both --moveit and
    --controller_name workspace, so if you want to run with moveit
    simply leave --controller_name as default.

    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, arbitrary.  Default: line'
    )
    parser.add_argument('-goal_point', '-g', type=float, default=[0, 0, 0], nargs=3, help=
        'Specify a goal point for line and circle trajectories. Ex: -g 0 0 0')
    parser.add_argument('-controller_name', '-c', type=str, default='jointspace', 
        help='Options: workspace, jointspace, or torque.  Default: jointspace'
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        (Hz) This specifies how many loop iterations should occur per second.  
        It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on 
        the real robot"""
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()

    if args.moveit:
        # We can't give moveit a workspace trajectory, so we make it
        # impossible.
        args.controller_name = 'jointspace'

    rospy.init_node('moveit_node')

    # this is used for sending commands (velocity, torque, etc) to the robot
    limb = baxter_interface.Limb(args.arm)


    # Choose an inverse kinematics solver. If you leave ik_solver as None, then the default
    # KDL solver will be used. Otherwise, you can uncomment the next line which sets ik_solver.
    # Then, the TRAC-IK inverse kinematics solver will be used. If you have trouble with inconsistency
    # or poor solutions with one method, feel free to try the other one.
    ik_solver = None
    # ik_solver = IK("base", args.arm + "_gripper")
    
    # A KDL instance for Baxter. This can be used for forward and inverse kinematics,
    # computing jacobians etc.
    kin = baxter_kinematics(args.arm)

    # This is a wrapper around MoveIt! for you to use.
    planner = PathPlanner('{}_arm'.format(args.arm))

    # Get an appropriate RobotTrajectory for the task (circular, linear, or arbitrary MoveIt)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace configurations and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(limb, kin, ik_solver, planner, args)

    if args.controller_name == "workspace":
        config = robot_trajectory.joint_trajectory.points[0].positions
        pose = create_pose_stamped_from_pos_quat(
            [config[0], config[1], config[2]],             # XYZ location
            [config[3], config[4], config[5], config[6]],  # XYZW quaterion orientation
            'base'
        )
        success, plan, time_taken, error_code = planner.plan_to_pose(pose)
        planner.execute_plan(plan)
    else:
        start = robot_trajectory.joint_trajectory.points[0].positions
        while not rospy.is_shutdown():
            try:
                # ONLY FOR EMERGENCIES!!! DON'T UNCOMMENT THE NEXT LINE UNLESS YOU KNOW WHAT YOU'RE DOING!!!
                # limb.move_to_joint_positions(joint_array_to_dict(start, limb), timeout=7.0, threshold=0.0001)
                success, plan, time_taken, error_code = planner.plan_to_joint_pos(start)
                planner.execute_plan(plan)
                break
            except moveit_commander.exception.MoveItCommanderException as e:
                print(e)
                print("Failed planning, retrying...")

    if args.moveit:
        # PROJECT 1 PART A
        # by publishing the trajectory to the move_group/display_planned_path topic, you should 
        # be able to view it in RViz.  You will have to click the "loop animation" setting in 
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        disp_traj.trajectory_start = RobotState()
        pub.publish(disp_traj)

        try:
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
        # the lines above will display the trajectory in RViz
        planner.execute_plan(robot_trajectory)
    else:
        # PROJECT 1 PART B
        from controllers.controllers import (
            WorkspaceVelocityController,
            PDJointVelocityController,
            PDJointTorqueController,
            FeedforwardJointVelocityController
        )
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory,
            rate=args.rate,
            timeout=args.timeout,
            log=args.log
        )
        if not done:
            print('Failed to move to position')
            sys.exit(0)


if __name__ == "__main__":
    main()
