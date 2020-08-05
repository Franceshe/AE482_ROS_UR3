import vrep
import time
import numpy as np
from numpy.linalg import multi_dot, norm, inv
from ece470_lib import *
import transforms3d
from usefulfuns2_siyunhe import *

'''
https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#transforms3d.euler.euler2mat
pip install transforms3d
'''

class Node:
    def __init__(self,theta,parent):
        self.theta = theta
        self.parent = parent


# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')
################################################################################


inital_S = []


# Dummies for Cuboids
result, dummy_0 = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_blocking)

# Get handle for the Cuboid
result, Cuboid_handle = vrep.simxGetObjectHandle(clientID, 'Cuboid', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for the Cuboid')

# Get position of the Cuboid
result, Cuboid_position = vrep.simxGetObjectPosition(clientID, Cuboid_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('Could not get object position for the Cuboid')
Cuboid_position = np.reshape(Cuboid_position,(3,1)) # Position of the cuboid
# print ("Cuboid position", Cuboid_position)

# Get the orientation from base to  world frame
result, Cuboid_orientation = vrep.simxGetObjectOrientation(clientID, Cuboid_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object orientation angles for UR3')

# Orientation of the cuboid
R_cuboid = transforms3d.euler.euler2mat(Cuboid_orientation[0], Cuboid_orientation[1], Cuboid_orientation[2])
# print ("R_cuboid", R_cuboid)

T_2in0 = np.block([
[R_cuboid[0,:], Cuboid_position[0,:]],
[R_cuboid[1,:], Cuboid_position[1,:]],
[R_cuboid[2,:], Cuboid_position[2,:]],
[0,0,0,1] ])
print ("T_2in0", T_2in0)

################################################################################

# Get joint object handles
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')

result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')

result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')

result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')

result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')

#Get handel for UR3
result, UR3_handle = vrep.simxGetObjectHandle(clientID, 'UR3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for UR3')
#Get handel for UR3_connection
result, UR3_connection = vrep.simxGetObjectHandle(clientID, 'UR3_connection', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for UR3UR3_connection')
################################################################################

#Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Get the orientation from base to  world frame
result, orientation = vrep.simxGetObjectOrientation(clientID, UR3_connection, UR3_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object orientation angles for UR3')

# Get the position from base to world frame
result, p = vrep.simxGetObjectPosition(clientID, UR3_connection, UR3_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
P_initial = np.reshape(p,(3,1))
# print ("P_initial", P_initial)
R_initial = transforms3d.euler.euler2mat(orientation[0], orientation[1], orientation[2])
# print ("R_itinial", R_initial)

M = np.block([
[R_initial[0,:], P_initial[0,:]],
[R_initial[1,:], P_initial[1,:]],
[R_initial[2,:], P_initial[2,:]],
[0,0,0,1] ])
# print ("M", M, "\n")
################################################################################

# Set up scew axis with respect to base frame
result, q1 = vrep.simxGetObjectPosition(clientID,joint_one_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q1 = np.reshape(q1,(3,1))
a1 = np.array([[0],[0],[1]])
S1 = toScrew(a1, q1)
# print ("q1", q1)
# print ("S1", S1)
inital_S.append(S1)

result, q2 = vrep.simxGetObjectPosition(clientID,joint_two_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q2 = np.reshape(q2,(3,1))
a2 = np.array([[-1],[0],[0]])
S2 = toScrew(a2, q2)
# print ("q2", q2)
# print ("S2", S2)
inital_S.append(S2)

result, q3 = vrep.simxGetObjectPosition(clientID,joint_three_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q3 = np.reshape(q3,(3,1))
a3 = np.array([[-1],[0],[0]])
S3 = toScrew(a3, q3)
# print ("q3", q3)
# print ("S3", S3)
inital_S.append(S3)


result, q4 = vrep.simxGetObjectPosition(clientID,joint_four_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q4 = np.reshape(q4,(3,1))
a4 = np.array([[-1],[0],[0]])
S4 = toScrew(a4, q4)
# print ("q4", q4)
# print ("S4", S4)
inital_S.append(S4)

result, q5 = vrep.simxGetObjectPosition(clientID,joint_five_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q5 = np.reshape(q5,(3,1))
a5 = np.array([[0],[0],[1]])
S5 = toScrew(a5, q5)
# print ("q5", q5)
# print ("S5", S5)
inital_S.append(S5)

result, q6 = vrep.simxGetObjectPosition(clientID,joint_six_handle, UR3_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object current position for UR3')
q6 = np.reshape(q6,(3,1))
a6 = np.array([[-1],[0],[0]])
S6 = toScrew(a6, q6)
# print ("q6", q6)
# print ("S6", S6)
inital_S.append(S6)

############### Get the theta values for joint variables #############################

result1, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result1 != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
# print('current value of first joint variable on UR3: theta = {:f}'.format(theta1))

result2, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result2 != vrep.simx_return_ok:
    raise Exception('could not get second joint variable')
# print('current value of second joint variable on UR3: theta = {:f}'.format(theta2))

result3, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result3 != vrep.simx_return_ok:
    raise Exception('could not get third joint variable')
# print('current value of third joint variable on UR3: theta = {:f}'.format(theta3))

result4, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result4 != vrep.simx_return_ok:
    raise Exception('could not get fourth joint variable')
# print('current value of fourth joint variable on UR3: theta = {:f}'.format(theta4))

result5, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result5 != vrep.simx_return_ok:
    raise Exception('could not get fifth joint variable')
# print('current value of fifth joint variable on UR3: theta = {:f}'.format(theta5))

result6, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result6 != vrep.simx_return_ok:
    raise Exception('could not get sixth joint variable')
# print('current value of sixth joint variable on UR3: theta = {:f}'.format(theta6))

#Inital thetas from the simulator
initial_thetas = [theta1, theta2, theta3, theta4, theta5, theta6]

################################################################################
# Let's use inverse kinematics to move the robot joints to grab the block

#parameters set up for creating dummy objects for joints and links

#Base dummy object
# body_names = ["Dummy_body_base"]
# body_diam = [0.4]

# #link is between two joints, we have 7 links in total
# link_names = ["Dummy_link_joint1", "Dummy_link_joint2", "Dummy_link_joint3", "Dummy_link_joint4", "Dummy_link_joint5", "Dummy_link_joint6", "Dummy_link_joint7"]

# #dummy objects for 6 joints
# joint_names = ["Dummy_joint1", "Dummy_joint2", "Dummy_joint3", "Dummy_joint4", "Dummy_joint5", "Dummy_joint6"]
# joint_diam = [0.3, 0.3, 0.2, 0.2, 0.15, 0.15]

# #ADD Base dummy to whole joints dummy.
# self_diam = body_diam.copy()
# self_diam.extend(joint_diam)
# link_centers = []
# body_centers = []
# joint_centers = []



#initialize parameter for goal pose
# x = 0.5
# y =  0.2
# z = 0.3
# alpha = -60
# beta  = 0
# gamma = 0

x = float(input("Enter an x-translation: "))
y = float(input("Enter an y-translation: "))
z = float(input("Enter an z-translation: "))
alpha =  int(input("Enter a rotation (in degrees) around the x-axis: "))
beta  =  int(input("Enter a rotation (in degrees) around the y-axis: "))
gamma =  int(input("Enter a rotation (in degrees) around the z-axis: "))


#ideally
# Enter an x-translation: 0.3
# Enter an y-translation: 1.5
# Enter an z-translation: 1
# Enter a rotation (in degrees) around the x-axis: 30
# Enter a rotation (in degrees) around the y-axis: 0
# Enter a rotation (in degrees) around the z-axis: 0


#inital scew axis
inital_S = np.asarray(inital_S)
inital_S = np.reshape(inital_S,(6,6))


goal = goalPose(x, y, z, alpha, beta, gamma)
# ik_thetas = inverseKinematics(goal, M, inital_S)
ik_thetas = findIK(goal, inital_S, M, theta=None, max_iter=100, max_err = 0.001, mu=0.05)
print("ik_thetas are", ik_thetas)


def moveUR3arm(thetas, clientID):
    #name of joint eg: 'UR3_joint1'
    armID = "UR3_" + "joint"

    for i in range(5):
        # Get "handle" to the a joint of the robot
        result, joint_handle = vrep.simxGetObjectHandle(clientID, armID + str(i+1), vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception("Could not get object handle for {} arm joint {}".format(1, i+1))

        # Set the desired value of the joint variable
        vrep.simxSetJointTargetPosition(clientID, joint_handle, thetas[i], vrep.simx_opmode_oneshot)

moveUR3arm(ik_thetas,clientID)

print("################################################################################")
################################################################################
################################################################################

start_pos = np.array([[0],[0],[0]])
end_pos = np.array([[M[0][3]],[M[1][3]],[M[2][3]]])
p_robot = np.block([[start_pos, q1, q2, q3, q4, q5, q6, end_pos]])
r_robot = np.array([[0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03]])
r_obstacle = np.array([[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]])

# Create p_obstacle array for collision checking and path planner

# Lists for final p_obstacle
entry_one = list()
entry_two = list()
entry_three = list()

# First, we handle first dummy (string combination), then we handle the rest in a loop
result, dummy_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_oneshot_wait)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for Dummy')

result, dummy_position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
if result != vrep.simx_return_ok:
    raise Exception('Could not get object position for Dummy')
entry_one.append(dummy_position[0])
entry_two.append(dummy_position[1])
entry_three.append(dummy_position[2])

# Loop for the rest
for i in range(11):
    dummy_handle_name = 'Dummy' + str(i)
    result, dummy_handle = vrep.simxGetObjectHandle(clientID, dummy_handle_name, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for ', dummy_handle_name)

    dummy_name = 'dummy_' + str(i+1)
    result, dummy_position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('Could not get object position for ', dummy_name)
    entry_one.append(dummy_position[0])
    entry_two.append(dummy_position[1])
    entry_three.append(dummy_position[2])
entry_one = np.asarray(entry_one)
entry_two = np.asarray(entry_two)
entry_three = np.asarray(entry_three)

#################### p_obstacle ################################################
p_obstacle = np.block([
[np.asarray(entry_one)], [np.asarray(entry_two)], [np.asarray(entry_three)]
])


#Path Planing Algorithm With Tree structure instead of linked list
# Find a path connects the start thetas to the goal thetas

def findPath(clientID, p_robot, r_robot, p_obstacle, r_obstacle):
    theta_start = Node(start,None)
    theta_goal = Node(goal,None)

    forward = [theta_start]
    backward = [theta_goal]

    n = 0
    while n <= 50:
        print(n)
        theta = np.zeros(6)
        # '''
        # Joint Bounds from Lab 3 Doc in degrees:
        # theta1 = (60,315)
        # theta2 = (-185,5)
        # theta3 = (-10,150)
        # theta4 = (-190,-80)
        # theta5 = (-120,80)
        # theta6 = (-180,180)
        # '''
        theta[0] = random.uniform(-np.pi,np.pi)
        theta[1] = random.uniform(-np.pi,np.pi)
        theta[2] = random.uniform(-np.pi,np.pi)
        theta[3]= random.uniform(-np.pi,np.pi)
        theta[4]= random.uniform(-np.pi,np.pi)
        theta[5] = random.uniform(-np.pi,np.pi)

        LeastDistanceSoFar = 10000000
        addedToforward = False
        addedTobackward = False
        step_size = 0.01

        for Tforward in forward:
            if np.linalg.norm(Tforward.theta-theta) < LeastDistanceSoFar:
                ClosestNode = Tforward
                LeastDistanceSoFar = np.linalg.norm(Tforward.theta-theta)


        if(point2point_collision_detector(theta, ClosestNode.theta, S, p_robot, r_robot, p_obstacle, r_obstacle, step_size) is 0):
            theta_new = Node(theta,ClosestNode)
            forward.append(theta_new)
            addedToforward = True

        LeastDistanceSoFar = 9999999999
        for Tbackward in backward:
            if np.linalg.norm(Tbackward.theta-theta) < LeastDistanceSoFar:
                ClosestNode = Tbackward
                LeastDistanceSoFar = np.linalg.norm(Tbackward.theta-theta)

        if(point2point_collision_detector(theta, ClosestNode.theta, S, p_robot, r_robot, p_obstacle, r_obstacle, step_size) is 0):
            theta_new_backward = Node(theta,ClosestNode)
            backward.append(theta_new_backward)
            addedTobackward = True


        if addedToforward and addedTobackward:
            path = [theta_new.theta]
            parent = theta_new.parent
            while parent != None:
                path = [parent.theta] + path
                parent = parent.parent
            
            parent = theta_new_backward.parent
            while parent != None:
                path = path + [parent.theta]
                parent = parent.parent

            return path

        n += 1

    return False

allpaths = findPath(clientID, p_robot, r_robot, p_obstacle, r_obstacle)
print(allpaths)







# Make a dummy for end_effector_pose
result, end_effector_pose = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)


time.sleep(2)
################################################################################
# Remove end_effector_pose dummy
vrep.simxRemoveObject(clientID, end_effector_pose, vrep.simx_opmode_oneshot_wait)

# Remove dummy handles for ConcretBlock1
vrep.simxRemoveObject(clientID, dummy_0, vrep.simx_opmode_oneshot_wait)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
