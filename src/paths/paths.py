#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa

Updated for Spring 2021 by Amay Saxena.

"""
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from utils.utils import *

try:
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except:
    pass

class MotionPath:
    def __init__(self, limb, kin, ik_solver, total_time):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.ik_solver = ik_solver
        self.total_time = total_time
        self.previous_computed_ik = None

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are the
        desired end-effector position, and the last four entries are the desired
        end-effector orientation as a quaternion, all written in the world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds to
        the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the arm's desired body-frame velocity at time t as a 6D twist. Note that this needs to
        be a rigid-body velocity, i.e. a member of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        pass

    def animate(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t)[:3] for t in times])
        
        def func(num, line):
            # NOTE: there is no .set_data() for 3 dim data...
            line.set_data(target_positions[:num, :2].T)
            line.set_3d_properties(target_positions[:num, 2].T)
            return line

        num_data_points = len(times)

        fig = plt.figure()
        ax = Axes3D(fig)
         
        # NOTE: Can't pass empty arrays into 3d version of plot()
        line = plt.plot(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], lw=2, c='g')[0] # For line plot
         
        # AXES PROPERTIES]
        # ax.set_xlim3d([limit0, limit1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("Trajectory evolution in robot's base frame.")
         
        # Creating the Animation object
        line_ani = animation.FuncAnimation(fig, func, frames=num_data_points, fargs=(line,), interval=50, blit=False)
        
        plt.show()

    def plot3d(self, num=300):
        fig = plt.figure()
        ax = Axes3D(fig)

        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t)[:3] for t in times])
        ax.plot3D(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], c='g')[0] # For line plot
        plt.show()

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t)[:3] for t in times])
        # Need to multiply by inverse of orientation to get the target X,Y,Z velocities in the
        # spatial frame rather than the body frame.
        target_velocities = np.vstack([
            np.matmul(get_g_matrix(self.target_position(t)[:3], self.target_position(t)[3:])[:3, :3].T,
                self.target_velocity(t)[:3]) for t in times])

        plt.figure()
        plt.subplot(3,2,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,2,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")
            
        plt.subplot(3,2,3)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,2,4)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")
            
        plt.subplot(3,2,5)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,2,6)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.show()

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()
        points = []
        for t in np.linspace(0, self.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace)
            points.append(point)

        # We want to make a final point at the end of the trajectory so that the 
        # controller has time to converge to the final point.
        extra_point = self.trajectory_point(self.total_time, jointspace)
        extra_point.time_from_start = rospy.Duration.from_sec(self.total_time + 1)
        points.append(extra_point)

        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a 
        RobotTrajectory.  
        
        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`


        joint_names: [left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2]
        points: 
        - 
        positions: [-0.11520713 -1.01663718 -1.13026189  1.91170776  0.5837694   1.05630898  -0.70543966]

        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            x_t, x_t_1, x_t_2 = None, None, None
            ik_attempts = 0
            theta_t_2 = self.get_ik(self.target_position(t-2*delta_t), seed=self.previous_computed_ik)
            theta_t_1 = self.get_ik(self.target_position(t-delta_t), seed=self.previous_computed_ik)
            theta_t   = self.get_ik(self.target_position(t), seed=self.previous_computed_ik)
            
            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here?
            point.positions = theta_t
            point.velocities = (theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (delta_t*delta_t)
            self.previous_computed_ik = theta_t
        else:
            point.positions = self.target_position(t)
            point.velocities = self.target_velocity(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point

    def get_ik(self, x, seed=None, ik_timeout=0.1, max_ik_attempts=10):
        """
        gets ik
        
        Parameters
        ----------
        x : 7x' :obj:`numpy.ndarray`
            pose of the end effector
        ik_timeout : float
            time in seconds after which ik solution will short circuit.

        Returns
        -------
        7x' :obj:`numpy.ndarray`
            joint values to achieve the passed in workspace position
        """
        if self.ik_solver and seed is None:
            seed = [0.0] * self.ik_solver.number_of_joints

        ik_attempts, theta = 0, None
        while theta is None and not rospy.is_shutdown():
            if self.ik_solver:
                theta = self.ik_solver.get_ik(seed,
                    x[0], x[1], x[2],      # XYZ
                    x[3], x[4], x[5], x[6] # quat
                    )
            else:
                theta = self.kin.inverse_kinematics(
                position=[x[0], x[1], x[2]],
                orientation=[x[3], x[4], x[5], x[6]])
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )
                print('MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x))
        return np.array(theta)

class LinearPath(MotionPath):
    def __init__(self, limb, kin, ik_solver, total_time):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, ik_solver, total_time)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are the
        desired end-effector position, and the last four entries are the desired
        end-effector orientation as a quaternion, all written in the world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds to
        the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        raise NotImplementedError

    def target_velocity(self, time):
        """
        Returns the arm's desired body-frame velocity at time t as a 6D twist. Note that this needs to
        be a rigid-body velocity, i.e. a member of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        raise NotImplementedError

class CircularPath(MotionPath):
    def __init__(self, limb, kin, ik_solver, total_time):
        """
        Remember to call the constructor of MotionPath
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, ik_solver, total_time)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are the
        desired end-effector position, and the last four entries are the desired
        end-effector orientation as a quaternion, all written in the world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds to
        the quaternion [0, 1, 0, 0].

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        raise NotImplementedError

    def target_velocity(self, time):
        """
        Returns the arm's desired body-frame velocity at time t as a 6D twist. Note that this needs to
        be a rigid-body velocity, i.e. a member of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        raise NotImplementedError

def verify_velocity(path):
    """
    (Requires scipy)

    Use this function to verify that your target_velocity implementation is consistent
    with your target_position implementation. This function will compute the target position
    over path.total_time seconds in two ways: first, by calling your target_position method,
    and second by integrating the output of your target_velocity method (using the matrix exponential,
    over SE(3)). It will then compare the output of those two methods at each timestep. It then collects
    the errors between the true onfiguration and the integrated configuration and prints out the mean
    error and standard deviation. It will also produce a 3D plot with the true trajectory (from target_positions)
    in red and the integrated trajectory in green. If your velocity implementation is correct, these plots should
    match exactly (if they match exactly, you will only really be able to see the green plot, since it will
    be printed over the red one). Also, the mean error should be very small.

    Note: For those curious, the error we use is a Frobenius error metric on SE(3). In particular, if we have two
    configurations g1 and g2, we compute the scalar "error" between them as d(g1, g2) = ||I - g1^-1 g2||_F
    where || . ||_F is the Frobenius matrix norm. This error is essentially a measure of how far the configuration
    g1^-1 g2 is from the identity (of course, if g1 = g2 then g1^-1 g2 = I, and hence d(g1, g2) = 0). Here, we 
    are using this to compare the SE(3) configuration returned by your target_position function to the one that 
    results from integrating the se(3) velocity returned by your target_velocity function.

    """
    import scipy.linalg
    initial_config = path.target_position(0)
    g0 = get_g_matrix(initial_config[:3], initial_config[3:])
    dt = 0.001
    times = np.arange(0.0, path.total_time, dt)
    gs = [g0]
    g = g0
    errors = []
    for t in times:
        true_config = path.target_position(t)
        g_true = get_g_matrix(true_config[:3], true_config[3:])
        error = np.linalg.norm(np.matmul(np.linalg.inv(g_true), g) - np.eye(4), ord='fro')
        errors.append(error)
        g = np.matmul(g, scipy.linalg.expm(dt * hat(path.target_velocity(t))))
        gs.append(g)

    print("Mean Frobenius Error:", np.mean(errors))
    print("Error Standard Deviation:", np.std(errors))

    # plt.plot(errors)
    # plt.show()

    gs = np.array(gs[:-1]) # remove last config as there is one extra after the previous loop.

    fig = plt.figure()
    ax = Axes3D(fig)

    positions = gs[:, :3, 3]
    true_positions = np.array([path.target_position(t) for t in times])

    ax.plot3D(true_positions[:, 0], true_positions[:, 1], true_positions[:, 2], c='r')
    ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2], c='g')

    plt.show()


if __name__ == '__main__':
    """
    Run this file to visualize plots of your paths. Note: the provided functions only
    visualize the end effector location, not its orientation. Use the plot3d function to 
    visualize the full trajectory in a 3D plot, use the animate function to see an animation
    of the trajectory as it evolves with time in 3D, and use the plot function to see the (X, Y, Z)
    components of the position and velocity (converted to the spatial frame) plotted against time.

    Finally, you can use the function verify_velocity to check that your velocity implementation
    is consistent with your position implementation.
    """

    path = LinearPath(TODO)
    path.animate()
    verify_velocity(path)
