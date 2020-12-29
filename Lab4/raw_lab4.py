import numpy as np
# measurements in meters

xw_grip = 0.2
yw_grip = -0.1
zw_grip = 0.2

x_g = xw_grip + 0.149
y_g = yw_grip - 0.149
z_g = zw_grip - 0.0193

d = 0.0535
theta_yaw = np.radians(45.0)

x_c = x_g - (d * np.cos(theta_yaw - np.radians(90)))
y_c = y_g + (d * np.sin(theta_yaw - np.radians(90)))
z_c = z_g

# Calculate theta_1
theta_x = np.arctan(y_c / x_c)
q = (0.120 - 0.093) + 0.083
d = np.sqrt((x_c**2) + (y_c**2))
theta_y = np.arcsin(q / d)

# Calculate theta_5, theta_6
theta_1 = theta_x - theta_y
theta_6 = np.radians(90) + theta_1 - theta_yaw
theta_5 = np.radians(-90)

# get x,y,z (end)
p1_0 = np.array([ [x_c], [y_c] ])
theta = np.radians(90) - theta_1
R1_0 = np.array([ [np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)] ])
p2_1 = np.array([ [0.11], [-0.083] ])
p2_0 = p1_0 + np.dot(R1_0, p2_1)
x_end = p2_0[0][0]
y_end = p2_0[1][0]
z_end = z_g + 0.052 + 0.082 # 0.082 in manual, 0.052 by hand
                            # it's from the end to the end of gripper
# print ("x_end:", x_end)
# print ("y_end:", y_end)
# print ("z_end:", z_end)

# Calculate theta_3
d1 = 0.152
a = np.sqrt((x_end**2) + (y_end**2) + ((z_end-d1)**2))
a2 = 0.244
a3 = 0.213
theta_3 = np.radians(180) - np.arccos(((a**2) - (a2**2) - (a3**2)) / (-2 * a2 * a3))

# Calculate theta_2
b = np.sqrt((x_end**2) + (y_end**2))
# gamma = np.arctan((z_end - d1) / b)
gamma = np.arccos(b/a)
theta_2 = -(np.arccos( (a2**2 + a**2 - a3**2) / 2*a2*a )) + gamma

# Calculate theta_4
alpha = np.arcsin((a3 * np.sin(np.radians(180) - theta_3))/a)
beta = np.radians(180) - (np.radians(180) - theta_3) - alpha
# theta_4 = -(np.radians(180) - (np.radians(90) - beta) - gamma)

# Print all theta's
# print ("Theta 1:", theta_1)
# print ("Theta_2:", theta_2)
# print ("Theta_3:", theta_3)
# print ("Theta_4:", theta_4)
# print ("Theta_5:", theta_5)
# print ("Theta 6:", theta_6)

################################################################################

# Theoretical values:
# ('Theta 1:', -1.01475)
# ('Theta_2:', -1.14021)
# ('Theta_3:', 1.28973)
# ('Theta_4:', -1.72031)
# ('Theta_5:', -1.5708)
# ('Theta 6:', -0.229352)
#
# Calculated values:
# ('Theta 1:', -1.0076629081093511)
# ('Theta_2:', -1.100946083769496)
# ('Theta_3:', 1.3018764656247861)
# ('Theta_4:', -1.8100212285309945)
# ('Theta_5:', -1.5707963267948966)
# ('Theta 6:', -0.22226474471190283)
