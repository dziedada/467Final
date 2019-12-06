import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
import math

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""
forearm_len = 0.10
upperarm_len = 0.10

R2D = 180.0/3.141592
D2R = 3.141592/180.0

angle_max = 120
angle_min = -120

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
    # print(len)
    A = np.empty((3, 3))
    s = math.sin(theta)
    c = math.cos(theta)

    R = [[c, s], [-s, c]]
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
    #print(joint_angles)
    for i in range(2, 0, -1):
        if i == 1 or i == 2:
            transform = calc_A_FK(0 - joint_angles[i], lens[i])

        pose = transform @ pose

    pose /= pose[2]
    # print(joint_angles)
    x = pose[0]
    y = pose[1]
    if x == 0.0:
        pose_angle = 0.0
    else:
        pose_angle = np.arctan(y / x) * R2D
    # phi = pose_angle + joint_angles[3]
    phi = pose_angle
    return [pose[0], pose[1], phi]

def check_valid(pose):
    """
    467TODO

    currently only angle limits are [-90, 90]
    """
    if pose[1] < 0:
        return False
    len = math.sqrt(pose[0] ** 2 + pose[1] ** 2)
    if len > forearm_len + upperarm_len or len < upperarm_len:
        print("Cannot reach: ", pose)
        return False

    return True

def IK(pose):
    """
    467TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    Assume pose = [x, y, phi] as in FK
    return angles for [base, shld, elbw]

    """

    # check if the target pose is valid
    if not check_valid(pose):
        return None
    x = pose[0]
    y = pose[1]
    target_len = math.sqrt(x ** 2 + y ** 2)
    
    A = np.arccos((upperarm_len ** 2 + target_len ** 2 - forearm_len ** 2) / (2 * upperarm_len * target_len))
    if y == 0.0:
        theta0 = - (np.pi / 2 + A)
    else:
        theta0 = - (np.arctan(abs(x / y)) + A)
    B = np.arccos((upperarm_len ** 2 + forearm_len ** 2 - target_len ** 2) / (2 * upperarm_len * forearm_len))    
    theta1 = math.pi - B
    if x < 0:
        theta0 = -theta0
        theta1 = -theta1
    # theta1 += math.pi / 2
    # if theta1 > math.pi:
    #     theta1 -= 2 * math.pi
    # print(A * R2D, B * R2D, theta0 * R2D, theta1 * R2D)    
    # check valid:
    if theta0 > angle_max * D2R or theta0 < angle_min * D2R:
        print(pose, " is out of upperarm's reach")
        return None
    if theta1 > angle_max * D2R and theta1 < angle_min * D2R:
        print(pose, " is out of forearm's reach")
        return None
    return [-math.pi / 2, theta0, theta1]


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