#include "lab4pkg/lab4.h"
#include <math.h>


// // helper function
// double dotProduct (double vect_A[], double vect_B[])
// {
// 	double product = 0;
// 	for(int i = 0; i < n; i++)
// 		product = product + vect_A[i] * vect_B[i];
// 	return product;
// }


//#define PI 3.14159265

/**
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{
// xw_grip = 0.2
// yw_grip = -0.1
// zw_grip = 0.2
//
// x_g = xw_grip + 0.149
// y_g = yw_grip - 0.149
// z_g = zw_grip - 0.0193
//
// d = 0.0535
// theta_yaw = np.radians(45.0)
//
// x_c = x_g - (d * np.cos(theta_yaw - np.radians(90)))
// y_c = y_g + (d * np.sin(theta_yaw - np.radians(90)))
// z_c = z_g
//
// # Calculate theta_1
// theta_x = np.arctan(y_c / x_c)
// q = (0.120 - 0.093) + 0.083
// d = np.sqrt((x_c**2) + (y_c**2))
// theta_y = np.arcsin(q / d)
//
// # Calculate theta_5, theta_6
// theta_1 = theta_x - theta_y
// theta_6 = np.radians(90) + theta_1 - theta_yaw
// theta_5 = np.radians(-90)
//
// # get x,y,z (end)
// p1_0 = np.array([ [x_c], [y_c] ])
// theta = np.radians(90) - theta_1
// R1_0 = np.array([ [np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)] ])
// p2_1 = np.array([ [0.11], [-0.083] ])
// p2_0 = p1_0 + np.dot(R1_0, p2_1)
// x_end = p2_0[0][0]
// y_end = p2_0[1][0]
// z_end = z_g + 0.052 + 0.082 # 0.082 in manual, 0.052 by hand
//                             # it's from the end to the end of gripper
// # print ("x_end:", x_end)
// # print ("y_end:", y_end)
// # print ("z_end:", z_end)
//
// # Calculate theta_3
// d1 = 0.152
// a = np.sqrt((x_end**2) + (y_end**2) + ((z_end-d1)**2))
// a2 = 0.244
// a3 = 0.213
// theta_3 = np.radians(180) - np.arccos(((a**2) - (a2**2) - (a3**2)) / (-2 * a2 * a3))
//
// # Calculate theta_2
// b = np.sqrt((x_end**2) + (y_end**2))
// # gamma = np.arctan((z_end - d1) / b)
// gamma = np.arccos(b/a)
// theta_2 = -(np.arccos( (a2**2 + a**2 - a3**2) / 2*a2*a )) + gamma
//
// # Calculate theta_4
// alpha = np.arcsin((a3 * np.sin(np.radians(180) - theta_3))/a)
// beta = np.radians(180) - (np.radians(180) - theta_3) - alpha
// # theta_4 = -(np.radians(180) - (np.radians(90) - beta) - gamma)
//
// # Print all theta's
// # print ("Theta 1:", theta_1)
// # print ("Theta_2:", theta_2)
// # print ("Theta_3:", theta_3)
// # print ("Theta_4:", theta_4)
// # print ("Theta_5:", theta_5)
// # print ("Theta 6:", theta_6)
//
// ################################################################################
//
// # Theoretical values:
// # ('Theta 1:', -1.01475)
// # ('Theta_2:', -1.14021)
// # ('Theta_3:', 1.28973)
// # ('Theta_4:', -1.72031)
// # ('Theta_5:', -1.5708)
// # ('Theta 6:', -0.229352)
// #
// # Calculated values:
// # ('Theta 1:', -1.0076629081093511)
// # ('Theta_2:', -1.100946083769496)
// # ('Theta_3:', 1.3018764656247861)
// # ('Theta_4:', -1.8100212285309945)
// # ('Theta_5:', -1.5707963267948966)
// # ('Theta 6:', -0.22226474471190283)


	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end;
	double xgrip,ygrip,zgrip;
	double gripper_len;
	double theta_yaw;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;

	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	d6 = (0.082+0.056);

	xgrip = xWgrip;
	ygrip = yWgrip;
	zgrip = zWgrip;

	xgrip += 0.149;
	ygrip -= 0.149;
	zgrip -= 0.0193;

	gripper_len = 0.0535;
	theta_yaw = PI*(yaw_WgripDegree/180);
	cout<<"theta_yaw: "<< theta_yaw<<endl;

	// xcen = xgrip - (gripper_len * cos(theta_yaw - PI/2));
	// ycen = ygrip + (gripper_len * sin(theta_yaw - PI/2));
	// zcen = zgrip;

	xcen = xgrip - (gripper_len * cos(theta_yaw * PI/180) );
	ycen = ygrip + (gripper_len * sin(theta_yaw * PI/180));
	zcen = zgrip;

	//Calculate theta_1
	double theta_x = atan(ycen / xcen);
	double q = (0.120 - 0.093) + 0.083;
	double d = sqrt((xcen*xcen) + (ycen*ycen));
	double theta_y = asin(q / d);

	//Calculate theta_5, theta_6
	theta1 = theta_x - theta_y;
	theta6 = PI/2 + theta1 - theta_yaw;
	theta5 = -PI/2;

	//# get x,y,z (end)
	double p1_0[] = {xcen, ycen};
	double theta = PI/2 - theta1;
	//double vect1[2] = {cos(theta), sin(theta)}
	//double vect2[2] = {-sin(theta), cos(theta)}
	double R1_0[2][2];
	R1_0[0][0] = cos(theta);
	R1_0[0][1] = sin(theta);
	R1_0[1][0] = -sin(theta);
	R1_0[1][1] = cos(theta);

	double p2_1[2];
	p2_1[0] = 0.11;
	p2_1[1] = -0.083;

	double temp[2] = {R1_0[0][0] * p2_1[0] + R1_0[0][1] * p2_1[1],
	R1_0[1][0] * p2_1[0] + 	R1_0[1][1] *  p2_1[1] };

	x3end = p1_0[0] + temp[0];
	y3end = p1_0[1] + temp[1];
	z3end = zgrip + 0.052 + 0.082; //# 0.082 in manual, 0.052 by hand
	                            //# it's from the end to the end of gripper

	// x3end = (sqrt(sqrt((xcen*xcen) + (ycen*ycen)) - (q*q)) - 0.083)*sin(theta1);
	// y3end = (sqrt(sqrt((xcen*xcen) + (ycen*ycen)) - (q*q)) - 0.083)*cos(theta1);
	// z3end = zcen;

    cout << x3end << '\n';
	cout << y3end << '\n';
	cout << z3end << '\n';

	cout << xcen << '\n';
	cout << ycen << '\n';
	cout << zcen << '\n';

	//Calculate theta_3
	double a = sqrt((x3end*x3end) + (y3end*y3end) + ((z3end-d1)*(z3end-d1)));
	theta3 = PI - acos(((a*a) - (a2*a2) - (a3*a3)) / (-2 * a2 * a3));

	//Calculate theta_2
	double b = sqrt((x3end*x3end) + (y3end*y3end));
	double gamma = atan2((z3end - d1), b);
	// double gamma = acos(b/a);
	cout << gamma << endl;
	theta2 = -(acos( ((a2*a2) + (a*a) - (a3*a3)) / 2*a2*a )) + gamma;

	//Calculate theta_4
	double alpha = asin((a3 * sin(PI - theta3))/a);
	double beta = PI - (PI - theta3) - alpha;
	theta4 = -(PI - (PI/2 - beta) - gamma);



	// theta2= -PI/4; // Default value Need to Change
	// theta3= PI/2; // Default value Need to Change
	// theta4= (-PI*3)/4; // Default value Need to Change




	// View values
	//use cout
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}
