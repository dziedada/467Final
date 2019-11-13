import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
import math

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""
forearm_len = 0.15
upperarm_len = 0.15

def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    pass

def calc_A_FK(theta, len):
    """
    467TODO:
    theta may be different from this assumption
    """

    A = np.empty((3, 3))
    s = math.sin(theta)
    c = math.cos(theta)

    R = [[len * c, len * s], [-len * s, len * c]]
    T = [len * s, len * c]

    A[:2, :2] = R
    A[:2, 2] = T
    A[2, :2] = 0.0
    A[2, 2] = 1.0

    return A

def FK_pox(joint_angles):
    """
    467TODO:
    joint_angles[1]: shld
    joint_angles[2]: elbw
    joint_angles[3]: wrst

    forearm_len, upperarm_len

    Calculate forward kinematics for rexarm
    
    return [x, y, phi], which is the pos of the end effector in a 2D world

    """
    
    lens = [0.0, upperarm_len, forearm_len]

    pose = np.array([0.0, 0.0, 1.0])
    for i in range(2, 0, -1):
        transform = calc_A_FK(joint_angles[i], lens[i])

        pose = transform @ pose

    pose /= pose[2]

    if x == 0.0:
        pose_angle = 0.0
    else:
        pose_angle = np.arctan(y / x) * R2D
    phi = pose_angle + joint_angles[3]

    return [pose[0], pose[1], phi]

def check_valid(pose):
    if pose[1] < 0:
        return False
    if pose[0] ** 2 + pose[1] ** 2 > forearm_len + upperarm_len:
        return False
    return True

def IK(pose):
    """
    467TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    Assume pose = [x, y, phi] as in FK
    return angles for [shld, elbw, wrst]

    """

    # check if the target pose is valid
    if not check_valid(pose):
        return None
    
    return [0, 0, 0]


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass