import numpy as np
import math
from numpy.linalg import norm

# ALL dim in meters
# Set of theta values:
#   theta1:10,      theta2:-60,     theta3:110,
#   theta4:-140,    theta5:-90,     theta6:10

# Predicted position of the tool frame:
x1 = 0.208
y1 = 0.325
z1 = 0.056

# Measured position of the tool frame:
x_mea_1 = 0.317618
y_mea_1 = 0.220702
z_mea_1 =  0.0551428

# Set of theta values:
#   theta1:-50,     theta2:-80,     theta3:130,
#   theta4:-130,    theta5:-90,     theta6:10

# Predicted position of the tool frame:
z2 = 0.071
x2 = 0.22
y2 = -0.1

# Measured position of the tool frame:
x_mea_2 =  0.281672
y_mea_2 = -0.0833532
z_mea_2 =  0.0703176

# Error Calculations
x1_arr_mea = np.array([[x_mea_1], [y_mea_1], [z_mea_1] ])
x1_arr = np.array([[x1], [y1], [z1] ])
x2_arr_mea = np.array([[x_mea_2], [y_mea_2], [z_mea_2] ])
x2_arr = np.array([[x2], [y2], [z2] ])
error_1 = norm(x1_arr_mea - x1_arr)
error_2 = norm(x2_arr_mea - x2_arr)

# Calculated error values:
print (error_1.tolist())
# Prints:   0.151310652367

print (error_2.tolist())
# Prints:   0.0638828396676

# Terminal output ROS:
#
# First set of angles
# youbot@ros06:~/catkin_muskula2$ rosrun lab3pkg lab3node 10 -60 110 -140 -90 10
# argc = 7
# theta1:10,theta2:-60,theta3:110,theta4:-140,theta5:-90,theta6:10
# Forward Kinematics calculated :
#   5.112e-08          -1 7.84328e-08    0.317618
#           1  2.2933e-08 8.37556e-08    0.220702
# -9.40114e-08  1.1714e-07           1   0.0551428
#           0           0           0           1
# driver msg destination:
# destination[]
#   destination[0]: 3.31613
#   destination[1]: -1.0472
#   destination[2]: 1.91986
#   destination[3]: -2.44346
#   destination[4]: -1.5708
#   destination[5]: 0.174533
# grip: 0
# softhome: 0
# duration: 0


# Second set of angles
# youbot@ros06:~/catkin_muskula2$ rosrun lab3pkg lab3node -50 -80 130 -130 -90 10
# argc = 7
# theta1:-50,theta2:-80,theta3:130,theta4:-130,theta5:-90,theta6:10
# Forward Kinematics calculated :
#    0.86433  -0.490383   0.111619   0.281672
#   0.502021   0.854564  -0.133022 -0.0833532
# -0.0301538    0.17101   0.984808  0.0703176
#          0          0          0          1
# driver msg destination:
# destination[]
#   destination[0]: 2.26893
#   destination[1]: -1.39626
#   destination[2]: 2.26893
#   destination[3]: -2.26893
#   destination[4]: -1.5708
#   destination[5]: 0.174533
# grip: 0
# softhome: 0
# duration: 0
