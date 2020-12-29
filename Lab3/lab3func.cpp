#include "lab3pkg/lab3.h"

/** translate DH parameters to Homogeneous Transformation Matrix
 *	Matrix4f is a type of 4x4 float numbered matrix.
 *  angles are in radian.
 *  distance are in meters.
 */
Matrix4f DH2HT(float a, float alpha, float d, float theta)
{
	Matrix4f HT;
	//you will need to write the HT matrix given a, alpha, d and theta
	HT<<cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta),
		sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta),
		0,sin(alpha),cos(alpha),d,
		0,0,0,1;
	//print out a, alpha, d, theta and HT matrix.
	#ifdef DEBUG
		cout<< "a: "<<a<<"; alpha: "<<alpha<<"; d: "<<d<<"; theta: "<<"; "<<endl;
		cout<<"HT matrix: "<<HT<<endl;
	#endif
	return HT; 
}


/** 
 * function that calculates encoder numbers for each motor.
 */
std::vector<double> lab_fk(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
	std::vector<double> return_value(6);

//Make sure to print text explaining all the parts of the Transformation Matrix
	//===========Implement joint angle to encoder expressions here.====================
	cout<<"Forward Kinematics calculated :"<<endl;

// Uncomment these when you have enter the correct value in each Question mark.
	cout<<	DH2HT(0, -PI/2, 152, theta1)*
			DH2HT(244,0,120, theta2)*
			DH2HT(213,0,-93, theta3)*
			DH2HT(0,-PI/2,83,theta4)*
			DH2HT(0, PI/2,83, theta5)*
			DH2HT(53,PI,145, theta6)<<endl;
	
	return_value[0]=theta1+PI;  // What the lab considers zero for Theta1 is 180 on the UR3 robot.
	return_value[1]=theta2;
	return_value[2]=theta3;
	return_value[3]=theta4;
	return_value[4]=theta5;
	return_value[5]=theta6;
	return return_value;
}