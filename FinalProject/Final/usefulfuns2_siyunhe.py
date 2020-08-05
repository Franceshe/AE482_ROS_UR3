import math
import numpy as np
from scipy.linalg import expm, logm
from numpy.linalg import multi_dot


# Get the skew symmetric matrix of an array
def skew(arr):
	mat = np.zeros((3,3))
	mat[0][1] = -1 * arr[2]
	mat[0][2] = arr[1]
	mat[1][0] = arr[2]
	mat[1][2] = -1 * arr[0]
	mat[2][0] = -1 * arr[1]
	mat[2][1] = arr[0]
	return mat

# Get the actual values from a skew-symmetric matrix
def unskew(mat):
	return np.array([[mat[2][1]], [mat[0][2]], [mat[1][0]]])

# Get the bracket of a screw
def bracket(v):
	Bracket = np.zeros((4,4))
	Bracket[0:3,0:3] = skew(v[0:3])
	Bracket[0:3,3] = np.transpose(v[3:])
	return Bracket

# Get the adjoint of a pose matrix
def adjoint(t):
	Ade = np.zeros((6,6))
	Ade[0:3,0:3] = t[0:3,0:3]
	Ade[3:,3:] = t[0:3,0:3]
	Ade[3:,0:3] = np.dot(skew(t[0:3,3]), t[0:3,0:3])
	return Ade

# Get the screw axis from the vector 'a' (a,b,c) that points along
# the axis of rotation and a point 'q' (d,e,f) that lies on the axis.
def screw(a,b,c,d,e,f):
	s = np.zeros(6)
	s[0:3] = np.array([a,b,c])
	s[3:] = -1*np.dot(skew(np.array([a,b,c])), np.array([d,e,f]))
	return s

# Turn a bracketed V into a twist
def twist(v):
	Twist = np.zeros(6)
	Twist[0:3] = np.transpose(unskew(v[0:3,0:3]))
	Twist[3:] = v[0:3,3]
	return Twist

# Compute a space jacobian
def spaceJacobian(S, theta):
	J = np.zeros((6,len(theta)))

	for i in range(len(theta)):
		if i == 0:
			J[:,i] = S[:,i]
		else:
			product = 1
			for j in range(i):
				product = np.dot(product, expm(bracket(S[:,j])*theta[j]))

			J[:,i] = np.dot(adjoint(product), S[:,i])

	return J

# Convert degrees to radians
def degToRad(angle):
	return angle * math.pi / 180

def goalPose(x, y, z, alpha, beta, gamma):
	pose = np.zeros((4,4))
	pose[0,3] = x
	pose[1,3] = y
	pose[2,3] = z
	pose[3,3] = 1

	x_rot = np.array([[1, 0, 0],
					  [0, math.cos(degToRad(alpha)), -1*math.sin(degToRad(alpha))],
					  [0, math.sin(degToRad(alpha)), math.cos(degToRad(alpha))]])

	y_rot = np.array([[math.cos(degToRad(beta)), 0, math.sin(degToRad(beta))],
					  [0, 1, 0],
					  [-1*math.sin(degToRad(beta)), 0, math.cos(degToRad(beta))]])

	z_rot = np.array([[math.cos(degToRad(gamma)), -1*math.sin(degToRad(gamma)), 0],
					  [math.sin(degToRad(gamma)), math.cos(degToRad(gamma)), 0],
					  [0, 0, 1]])

	rotation = np.dot(np.dot(x_rot, y_rot), z_rot)
	pose[0:3,0:3] = rotation
	return pose


def forwardKinematics(M, S, thetas):

	product = 1
	for s in range(len(thetas)):
		# print((bracket(S[:,s])).shape)
		product = np.dot(product, expm(bracket(S[:,s])*thetas[s]))

	T = np.dot(product, M)
	return T


def findIK(endT, S, M, theta=None, max_iter=100, max_err = 0.001, mu=0.05):
    """
    Basically Inverse Kinematics
    Uses Newton's method to find joint vars to reach a given pose for a given robot. Returns joint positions and 
    the error. endT, S, and M should be provided in the space frame. Stop condiditons are when the final pose is less than a given
    twist norm from the desired end pose or a maximum number of iterations are reached. 
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix) 
    TODO: Improve internal type flexibilty of input types
    :param endT: the desired end pose of the end effector
    :param S: a python list of 6x1 screw axes in the space frame
    :param M: the pose of the end effector when the robot is at the zero position
    :param theta: Optional - An initial guess of theta. If not provided, zeros are used. Should be a Nx1 numpy matrix
    :param max_iter: Optional - The maximum number of iterations of newtons method for error to fall below max_err. Default is 10
    :param max_err: Optional - The maximum error to determine the end of iterations before max_iter is reached. Default is 0.001 and should be good for PL/quizes
    :param mu: The normalizing coefficient (?) when computing the pseudo-inverse of the jacobian. Default is 0.05
    :returns: A tuple where the first element is an Nx1 numpy array of joint variables where the algorithm ended. Second 
              element is the norm of the twist required to take the found pose to the desired pose. Essentially the error that PL checks against.
    """
    if  theta is None:
        theta = np.zeros((len(S),1))
    V = np.ones((6,1))
    while np.linalg.norm(V) > max_err and max_iter > 0:
        curr_pose = evalT(S, theta, M)
        V = inv_bracket(logm(endT.dot(inv(curr_pose))))
        J = evalJ(S, theta)
        pinv = inv(J.transpose().dot(J) + mu*np.identity(len(S))).dot(J.transpose())
        thetadot = pinv.dot(V)
        theta = theta + thetadot
        max_iter -= 1;
    return (theta, np.linalg.norm(V))

def inverseKinematics(goal, M, S):
	#initial gase of thetas
	theta = np.random.rand(S.shape[1])
	V_error = 5
	theta_error = 5
	
	count = 0
	while V_error > 0.1 or theta_error > 0.01:
		if count > 300:
			return theta

		#Goal Pose
		T = forwardKinematics(M, S, theta)

		V_bracket = logm(np.dot(goal, np.linalg.inv(T)))

		#Compute Required spatial twist to achieve this
		V = twist(V_bracket)

		#calculate the space jacobian
		J = spaceJacobian(S, theta)

		# Theta = Theta + [ (JT * J + 0.00001*I)^-1 * (JT * V) ] - [ (I - J#J) * Theta ]
		theta_dot = np.dot(np.linalg.inv(np.dot(np.transpose(J), J) + 0.1*np.identity(1)), 
			np.dot(np.transpose(J), V)) 
		- np.dot(np.identity(1) - np.dot(np.linalg.pinv(J), J), theta)
		# theta_dot = np.dot(J, V)
		theta = theta + theta_dot
		V_error = np.linalg.norm(V)
		theta_error =  np.linalg.norm(theta_dot)
		print("V Error: {}, Theta Error: {}".format(V_error, theta_error))
		count += 1

	return theta

#Inhereited from check_point4_siyunhe2
#Helper functions for collision checking while path finding
def updateCenterSphere(clientID, centers, S, thetas):
    new_centers = []
    thetas = thetas.copy()

    joints_to_add = [0,1,2,3,4,5]
    for i in range(5):
        old_position = np.block([centers[i], 1])
        new_position = forwardKinematics(old_position, S[:,:joints_to_add[i]+1], thetas[:joints_to_add[i]+1])
        new_centers.append(new_position[0:3])
    return new_centers

# Check for collision
def checkCollision(joint_centers, body_centers, link_centers):

    # Check for link collision
    for i in range(len(joint_names)):
        center = joint_centers[i]

        for j in range(len(link_names)):
            link = link_centers[j]

            if np.linalg.norm(center - link) < joint_diam[i]/2 + link_diam/2:
                return True

    # Check for self-collision
    self_centers = body_centers.copy()
    self_centers.extend(joint_centers)
    total = len(joint_names) + len(body_names)
    for i in range(total):
        for j in range(total-1-i):
            if np.linalg.norm(self_centers[i] - self_centers[j+i+1]) < self_diam[i]/2 + self_diam[j+i+1]/2:
                return True

    return False

#from check_point_5 helper functions
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

# def checkStraightLine(clientID, theta_a, theta_b, arm_centers, body_centers, rack_centers, SLeft, SRight, arm):
# 	dtheta = 0.01
# 	s = 0
# 	while s <= 1:
# 		theta = (1-s)*theta_a + s*theta_b
# 		if arm == "left":
# 			theta = np.block([theta,0,0,0,0,0,0,0])
# 			new_arm_centers = updateCenters(clientID, arm_centers, SLeft, SRight, theta)
# 		else:
# 			theta = np.block([theta[0],0,0,0,0,0,0,0,theta[1:]])
# 			new_arm_centers = updateCenters(clientID, arm_centers, SLeft, SRight, theta)
# 		collision = checkCollision(new_arm_centers, body_centers, rack_centers)
# 		if collision:
# 			return True
# 		s = s + dtheta
# 	return False

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

