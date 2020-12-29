#include "lab4pkg/lab4.h"
#include <math.h>

/**
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{
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
	theta_yaw = PI*((yaw_WgripDegree-90)/180);
	cout<<"theta_yaw: "<< theta_yaw<<endl;

	xcen = xgrip - (gripper_len * cos(theta_yaw - PI/2));
	ycen = ygrip + (gripper_len * sin(theta_yaw - PI/2));
	zcen = zgrip;


	//Calculate theta_1
	double theta_x = atan2(ycen, xcen);
	double q = (0.120 - 0.093) + 0.083;
	double d = sqrt((xcen*xcen) + (ycen*ycen));
	double theta_y = asin(q / d);

	//Calculate theta_5, theta_6
	theta1 = theta_x - theta_y;
	theta6 = theta1 - theta_yaw;
	theta5 = -PI/2;

	//# get x,y,z (end)
	double p1_0[] = {xcen, ycen};
	double theta = PI/2 - theta1;
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
	z3end = zgrip + 0.052 + 0.082; //0.082 in manual, 0.052 by hand
	                               // it's from the end to the end of gripper


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
	//aplha that was just used for solving theta 4 can be used here too
	double alpha = asin((a3 * sin(PI - theta3))/a);
	//theta2 = -(acos( ((a2*a2) + (a*a) - (a3*a3)) / 2*a2*a )) + gamma;
	theta2 = -1*(gamma + alpha);

	//Calculate theta_4
	//Not changed
	double beta = PI - (PI - theta3) - alpha;
	theta4 = -(PI - (PI/2 - beta) - gamma);

	//The issues were maybe with (xcen, ycen), theta_yaw conversion,
	//but most likely with theta2 calculation

	//Input:  x = 0.2, y = 0.1, z=0.2, theta_yaw = 45 degrees
	//Outout: x = 0.211, y = 0.093, z = 0.174, theta_yaw = 43 degrees

	//Input:  x = 0.2, y = -0.1, z=0.2, theta_yaw = -45 degrees
	//Outout: x = 0.197, y = -0.075, z = 0.175, theta_yaw = -43 degrees


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
