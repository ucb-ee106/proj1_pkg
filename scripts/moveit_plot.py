#!/usr/bin/env python

import rospy
from utils.utils import *
import matplotlib.pyplot as plt
import baxter_interface
from baxter_pykdl import baxter_kinematics
from proj1_sols_pkg.srv import TriggerLogging, TriggerLoggingResponse
from moveit_msgs.msg import RobotTrajectory
import sys

class MoveIt_Plot:
    def __init__(self, arm, rate):
        self.start_going = False
        self.keep_going = True
        self._path = None
        self._limb = baxter_interface.Limb(arm)
        self._kin = baxter_kinematics(arm)
        self._rate = rate
        rospy.Service('start_logging', TriggerLogging, self.start_logging)

    def start_logging(self, req):
        self._path = req.path
        self.start_going = True
        return TriggerLoggingResponse()

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time
        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search
        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)
    
    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace
        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))
        actual_workspace_quaternions = np.zeros((len(times), 4))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            fk = self._kin.forward_position_kinematics(joint_values=positions_dict)
            
            actual_workspace_positions[i, :] = fk[:3]
            actual_workspace_velocities[i, :] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
            actual_workspace_quaternions[i, :] = fk[3:]
        # it's joint space

        target_workspace_positions = np.zeros((len(times), 3))
        target_workspace_velocities = np.zeros((len(times), 3))
        target_workspace_quaternions = np.zeros((len(times), 4))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(target_positions[i], self._limb)
            fk = self._kin.forward_position_kinematics(joint_values=positions_dict)
            target_workspace_positions[i, :] = fk[:3]
            target_workspace_velocities[i, :] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])
            target_workspace_quaternions[i, :] = np.array(fk[3:])

        # Plot joint space
        plt.figure()
        # print len(times), actual_positions.shape()
        joint_num = len(self._limb.joint_names())
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_positions[:,joint], label='Actual')
            plt.plot(times, target_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Joint " + str(joint) + " Position Error")
            plt.legend()

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Joint " + str(joint) + " Velocity Error")
            plt.legend()
        print("Close the plot window to continue")
        plt.show()
        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")
            plt.legend()

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")
            plt.legend()

        print("Close the plot window to continue")
        plt.show()

        # Plot orientation error. This is measured by considering the
        # axis angle representation of the rotation matrix mapping
        # the desired orientation to the actual orientation. We use
        # the corresponding angle as our metric. Note that perfect tracking
        # would mean that this "angle error" is always zero.
        angles = []
        for t in range(len(times)):
            quat1 = target_workspace_quaternions[t]
            quat2 = actual_workspace_quaternions[t]
            theta = axis_angle(quat1, quat2)
            angles.append(theta)

        plt.figure()
        plt.plot(times, angles)
        plt.xlabel("Time (s)")
        plt.ylabel("Error Angle of End Effector (rad)")
        print("Close the plot window to continue")
        plt.show()
        
    def run(self):

        r = rospy.Rate(1)
        while not self.start_going:
            r.sleep()

        times = list()
        actual_positions = list()
        actual_velocities = list()
        target_positions = list()
        target_velocities = list()

        # For interpolation
        max_index = len(self._path.joint_trajectory.points)-1
        current_index = 0

        r = rospy.Rate(self._rate)
        start_t = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(self._path, t, current_index)

            # For plotting
            times.append(t)
            actual_positions.append(current_position)
            actual_velocities.append(current_velocity)
            target_positions.append(target_position)
            target_velocities.append(target_velocity)

            if current_index >= max_index:
                break

            r.sleep()
        
        self.plot_results(
            times, 
            actual_positions, 
            actual_velocities, 
            target_positions, 
            target_velocities)


if __name__ == "__main__":
    rospy.init_node("moveit_plot", anonymous=True)
    arm = sys.argv[1]
    rate = int(sys.argv[2])
    moveit_plot = MoveIt_Plot(arm, rate)
    moveit_plot.run()
