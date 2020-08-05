import numpy as np
import math
import random
from scipy.linalg import expm, logm
from numpy.linalg import inv, multi_dot, norm
from ece470_lib import *


class Node(object):
    def __init__(self):
        self.theta = None
        self.next = None

def print_linked_list(Node):
    while Node is not None:
        Node = Node.next
#-1 returned means the node arguement passed in was NULL
def find_closest_node(node, theta_target):
    distance = -1
    closest_node = node

    while (node is not None):
        if(norm(node.theta - theta_target) < distance or distance is -1):
            distance = norm(node.theta - theta_target)
            closest_node = node

        node = node.next

    return closest_node

#None means no path exists
def get_complete_path(start, end):
    intersecting_theta = None
    curr_node_s = start

    while curr_node_s is not None:
        curr_node_e = end
        flag = False
        while curr_node_e is not None:
            if(norm(curr_node_s.theta - curr_node_e.theta) == 0):
                flag = True
                intersecting_theta = curr_node_s.theta
                break
            curr_node_e = curr_node_e.next

        if(flag):
            break

        curr_node_s = curr_node_s.next

    #No shared theta so return None
    if(intersecting_theta is None):
        return intersecting_theta

    q = list()
    backtrack = dict()

    #puts thetas from start till the shared theta, including the shared theta
    curr_node_s = start
    while curr_node_s is not None:
        q.append(curr_node_s.theta)

        if(norm(curr_node_s.theta - intersecting_theta) == 0):
            break

        curr_node_s = curr_node_s.next

    #puts the thetas from the shared theta till the end, not including the shared data
    curr_node_e = end
    flag = False
    while curr_node_e is not None:

        if(flag):
            q.append(curr_node_e.theta)

        if(norm(curr_node_e.theta - intersecting_theta) == 0):
            flag = True

        curr_node_e = curr_node_e.next

    return q

def self_collision_detector(p, r):
    for i in range(len(p[0])):
        point1 = np.array([
        [p[0][i]],
        [p[1][i]],
        [p[2][i]]
        ])

        for j in range((len(p[0])) - i):
            if(i == i+j):
                continue

            point2 = np.array([
            [p[0][j+i]],
            [p[1][j+i]],
            [p[2][j+i]]
            ])
            # print(i,j)
            if(norm(point1 - point2) <= r[i]+r[i+j]):
                return 1
    return 0


def other_collision_detector(robot, others, r_robot, r_others):
    for i in range(len(robot[0])):
        point1 = np.array([
        [robot[0][i]],
        [robot[1][i]],
        [robot[2][i]]
        ])

        if(point1[2][0] <= 0 and i != 0):
            return 1

        for j in range(len(others[0])):
            point2 = np.array([
            [others[0][j]],
            [others[1][j]],
            [others[2][j]]
            ])

            # print(norm(point1 - point2), r_robot[i]+r_others[j])
            if(norm(point1 - point2) <= r_robot[i] + r_others[j]):
                return 1
    return 0


def point2point_collision_detector(theta_a, theta_b, S, p_robot, r_robot, p_obstacle, r_obstacle, step_size):
    s = step_size
    while s <= 1:
        theta = (1-s)*theta_a + s*theta_b

        final_pos = finalpos(S, theta, p_robot)
        self_colision_flag = self_collision_detector(final_pos, r_robot[0])
        other_colision_flag = other_collision_detector(final_pos, p_obstacle, r_robot[0], r_obstacle[0])

        if(self_colision_flag or other_colision_flag):
            return s
        s = s + step_size
    return 0

def user_input():
    x = float(input("Enter X translation position: "))
    y = float(input("Enter Y translation position: "))
    z = float(input("Enter Z translation position: "))
    a = float(input("Enter a rotational angle in degrees: "))
    b = float(input("Enter b rotational angle in degrees: "))
    c = float(input("Enter c rotational angle in degrees: "))
    Goal_pose = RotationMatrixToPose(x, y, z, a, b, c)
    return Goal_pose


def RotationMatrixToPose(x, y, z, a, b, c):
    Goal_pose = np.zeros((4,4))
    Goal_pose[0,3] = x
    Goal_pose[1,3] = y
    Goal_pose[2,3] = z
    Goal_pose[3,3] = 1

    Rot_x = np.array([[1, 0, 0],
                      [0, math.cos(deg2rad(a)), -1*math.sin(deg2rad(a))],
                      [0, math.sin(deg2rad(a)), math.cos(deg2rad(a))]])

    Rot_y = np.array([[math.cos(deg2rad(b)), 0, math.sin(deg2rad(b))],
                      [0, 1, 0],
                      [-1*math.sin(deg2rad(b)), 0, math.cos(deg2rad(b))]])

    Rot_z = np.array([[math.cos(deg2rad(c)), -1*math.sin(deg2rad(c)), 0],
                      [math.sin(deg2rad(c)), math.cos(deg2rad(c)), 0],
                      [0, 0, 1]])


    R = multi_dot([Rot_x, Rot_y, Rot_z])
    Goal_pose[0:3,0:3] = R
    return Goal_pose


def euler_to_a(a,b,g):
    R = euler2mat(a,b,g)
    a = np.reshape(R[:,2],(3,1))
    return a


def euler2mat(z=0, y=0, x=0):
    Ms = []
    if z:
        cosz = math.cos(z)
        sinz = math.sin(z)
        Ms.append(np.array(
                [[cosz, -sinz, 0],
                 [sinz, cosz, 0],
                 [0, 0, 1]]))
    if y:
        cosy = math.cos(y)
        siny = math.sin(y)
        Ms.append(np.array(
                [[cosy, 0, siny],
                 [0, 1, 0],
                 [-siny, 0, cosy]]))
    if x:
        cosx = math.cos(x)
        sinx = math.sin(x)
        Ms.append(np.array(
                [[1, 0, 0],
                 [0, cosx, -sinx],
                 [0, sinx, cosx]]))
    if Ms:
        return reduce(np.dot, Ms[::-1])
    return np.eye(3)

def skew(R):
	a = R[0]
	b = R[1]
	c = R[2]
	skew_a = np.array([[0 ,-c ,b] , [c ,0 ,-a], [-b, a ,0]])
	return skew_a


def bracket_skew_4(x):
    return np.array([[0,-x[2],x[1],x[3]],
                     [x[2],0,-x[0],x[4]],
                     [-x[1],x[0],0,x[5]],
                     [0,0,0,0]])


def revolute(a,q):
    aq = -np.dot(skew(a), q)
    screw = np.array([
    [a[0][0]],
    [a[1][0]],
    [a[2][0]],
    [aq[0][0]],
    [aq[1][0]],
    [aq[2][0]]
    ])
    return screw


def exp_s_the(s,theta):
	return expm(bracket_skew_4(s)*theta)


def deg2rad(deg):
	return deg * (np.pi) / 180


def rad2deg(rad):
    return rad * 180 / (np.pi)


def euler_to_axis(x):
	return euler_to_a(deg2rad(x[0]), deg2rad(x[1]), deg2rad(x[2]))


def rot2euler(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if np.isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif np.isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi



def screwBracForm(mat) :

     brac = np.array([
     [0, -1 * mat[2][0], mat[1][0], mat[3][0]],
     [mat[2][0], 0, -1 * mat[0][0], mat[4][0]],
     [-1 * mat[1][0], mat[0][0], 0, mat[5][0]],
     [0,0,0,0]
     ])
     return brac

def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q
