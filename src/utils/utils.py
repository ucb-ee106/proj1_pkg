#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
from math import sin, cos, atan2
import itertools

try:
    from geometry_msgs.msg._Point import Point
    import tf.transformations as tfs
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ros_enabled = True
except:
    ros_enabled = False


def convert_jointspace_to_workspace_trajectory(joint_traj, limb, kin):
    """
    PROJECT 1 PART B

    Converts a jointspace RobotTrajectory into a workspace trajectory.

    Recall that for workspace trajectories, we store positions as SE(3)
    configurations given in the form of a 7D vector, where the first 3 entries are
    the spatial x,y,z position, and the final 4 entries are the quaternion representation
    of the orientation.
    """
    raise NotImplementedError

    traj = JointTrajectory()
    traj.joint_names = limb.joint_names()
    points = []
    print("Length of trajectory:", len(joint_traj.joint_trajectory.points))

    for i in range(len(joint_traj.joint_trajectory.points) - 1):
        curr_pos = joint_traj.joint_trajectory.points[i].positions
        next_pos = joint_traj.joint_trajectory.points[i + 1].positions

        # Compute workspace configuration and body velocity.
        # hint: kin.forward_position_kinematics may be useful.
        curr_workspace_config = None
        body_velocity = None

        point = JointTrajectoryPoint()
        point.positions = curr_workspace_config
        point.velocities = body_velocity
        point.time_from_start = joint_traj.joint_trajectory.points[i].time_from_start

        points.append(point)

    # We want to make a final point at the end of the trajectory so that the 
    # controller has time to converge to the final point.
    last_point = JointTrajectoryPoint()
    last_pos = joint_traj.joint_trajectory.points[-1].positions
    last_time = joint_traj.joint_trajectory.points[-1].time_from_start

    positions_dict = joint_array_to_dict(last_pos, limb)
    last_workspace_config = kin.forward_position_kinematics(joint_values=positions_dict)

    last_point.positions = last_workspace_config

    # What should the desired body velocity at this final position?
    last_point.velocities = None

    last_point.time_from_start = last_time

    points.append(last_point)

    traj.points = points
    traj.header.frame_id = 'base'
    robot_traj = RobotTrajectory()
    robot_traj.joint_trajectory = traj
    return robot_traj

def g_matrix_log(g):
    """
    PROJECT 1 PART B
    
    Implements the matrix logarithm on SE(3).
    Returns a 6D vector xi such that g = exp(hat(xi)).

    Hint: the function R_matrix_log has already been implemented for you.
    """
    R, p = g[:3, :3], g[:3, 3]
    w = R_matrix_log(g[:3, :3])

    if np.allclose(w, 0):
        # omega = 0 case.
        xi = np.zeros(6)
        xi[:3] = p
    else:
        theta = np.linalg.norm(w)
        w_unit = w / np.linalg.norm(w)

        # Your code here
    
    return xi

def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.linalg.norm(vec)

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec : nx' :obj:`numpy.ndarray

    Returns
    -------
    nx' :obj:`numpy.ndarray`
    """
    return vec / length(vec)

def joint_array_to_dict(vel_torque_array, limb):
    """
    the baxter interface requires you to send the joint velocities / torques
    as a dictionary, this turns and array of velocities and torques into a 
    dictionary with joint names.

    Parameters
    ----------
    vel_torque_array : 7x' :obj:`numpy.ndarray`
        numpy array of velocities or torques to be sent to the baxter
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    :obj:`dict` of string->float
        mapping of joint names to joint velocities / torques
    """

    return dict(zip(limb.joint_names(), vel_torque_array))

def get_joint_positions(limb):
    """
    Returns the baxter joint positions IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint positions
    """
    return np.array([limb.joint_angles()[joint_name] for joint_name in limb.joint_names()])

def get_joint_velocities(limb):
    """
    Returns the baxter joint velocities IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint velocities
    """
    return np.array([limb.joint_velocities()[joint_name] for joint_name in limb.joint_names()])

def vec(*args):
    """
    all purpose function to get a numpy array of random things.  you can pass
    in a list, tuple, ROS Point message.  you can also pass in:
    vec(1,2,3,4,5,6) which will return a numpy array of each of the elements 
    passed in: np.array([1,2,3,4,5,6])
    """
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif ros_enabled and type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v : :obj:`numpy.ndarrray`
        vector form of shape 3x1, 3x, 6x1, or 6x

    Returns
    -------
    3x3 or 6x6 :obj:`numpy.ndarray`
        hat version of the vector v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def vee(mat):
    """
    Inverse of the hat operation. If the input is 3x3, assumes input
    is a member of so(3) and returns a 3D vector. If the input is 4x4,
    assumes the input is a member of se(3) and returns a 6D vector.
    """
    if mat.shape == (3, 3):
        return np.array([mat[2, 1], mat[0, 2], mat[1, 0]])
    elif mat.shape == (4, 4):
        return np.array([mat[0, 3], mat[1, 3], mat[2, 3], mat[2, 1], mat[0, 2], mat[1, 0]])
    else:
        raise ValueError

def axis_angle(quat1, quat2):
    """
    Computes the angle between the configurations described by the
    two quaternions.
    """
    R1 = tfs.quaternion_matrix(quat1)[:3, :3]
    R2 = tfs.quaternion_matrix(quat2)[:3, :3]
    R = np.matmul(R1.T, R2) 
    tr_R = sum(R[i, i] for i in range(3))
    theta = np.arccos((tr_R - 1) / 2.0)
    return theta

def R_matrix_log(R):
    """
    Implements the matrix logarithm on SE(3).
    Returns a 3D vector omega such that R = exp(hat(omega)).
    """
    if np.allclose(R, np.eye(3)):
        return np.zeros(3)
    theta = np.arccos((np.trace(R) - 1) / 2.0)
    omega_unit = (1 / (2 * np.sin(theta))) * vee(R - R.T)
    return omega_unit * theta

def get_g_matrix(position, quat):
    g = np.eye(4)
    g[:3, :3] = tfs.quaternion_matrix(quat)[:3, :3]
    g[:3, 3] = position
    return g

def g_inv(g):
    """
    Inverse of a 4x4 rigid body transformation g.
    """
    g_inv = np.eye(4)
    R = g[:3, :3]
    p = g[:3,  3]
    g_inv[:3, :3] = R.T
    g_inv[:3,  3] = -np.matmul(R.T, p)
    return g_inv 

def adj(g):
    """
    Adjoint of a rotation matrix.  See the MLS book

    Parameters
    ----------
    g : 4x4 :obj:`numpy.ndarray`
        Rotation matrix

    Returns
    -------
    6x6 :obj:`numpy.ndarray` 
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = np.matmul(hat(p), R)
    result[3:6,3:6] = R
    return result

def look_at_general(origin, direction):
    """
    Creates a 3D Rotation Matrix at the origin such that the z axis is the same
    as the direction specified.  There are infinitely many of such matrices, 
    but we choose the one where the x axis is as vertical as possible.  

    Parameters
    ----------
    origin : 3x1 :obj:`numpy.ndarray`
    direction : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    up = vec(0,0,1)
    z = normalize(direction)
    x = normalize(np.cross(up, z))
    y = np.cross(z, x) 

    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin
    return result

def create_pose_from_rigid_transform(g):
    """
    takes a rotation matrix and turns it into a ROS Pose

    Parameters
    ----------
    g : 4x4 : :obj:`numpy.ndarray`

    Returns
    -------
    :obj:`geometry_msgs.msg.Pose`
    """
    position = tfs.translation_from_matrix(g)
    quaternion = tfs.quaternion_from_matrix(g)
    wpose = Pose()
    wpose.position.x = position[0]
    wpose.position.y = position[1]
    wpose.position.z = position[2]
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose

def create_pose_stamped_from_pos_quat(pos, quat, frame_id):
    """
    takes a position and quaternion and turns it into a ROS PoseStamped

    Parameters
    ----------
    pos : 3x1 : :obj:`numpy.ndarray`
    quat : 4x1 : :obj:`numpy.ndarray`


    Returns
    -------
    :obj:`geometry_msgs.msg.PoseStamped`
    """
    wpose = PoseStamped()
    wpose.header.frame_id = frame_id
    wpose.pose.position.x = pos[0]
    wpose.pose.position.y = pos[1]
    wpose.pose.position.z = pos[2]
    wpose.pose.orientation.x = quat[0]
    wpose.pose.orientation.y = quat[1]
    wpose.pose.orientation.z = quat[2]
    wpose.pose.orientation.w = quat[3]
    return wpose
