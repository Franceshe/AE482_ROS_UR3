/********************LAB 4.cpp*************
	only contains main function.
	The forward kinematics function is implemented in lab4func.cpp
*/


#include "lab4pkg/lab4.h"

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

int main(int argc, char **argv)
{
	//initialization & variable definition
	ros::init(argc, argv, "lab4node");
	ros::NodeHandle nh;
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	ece470_ur3_driver::command driver_msg;

	//command line parse:
	if(argc!=5){
		cout<<"invalid number of input"<<endl;
		cout<<"command should be rosrun lab# lab# xWgrip yWgrip zWgrip yaw_WgripDegree"<<endl;
		return 0;
	}
	//float xWgrip, float yWgrip, float zWgrip, float yaw
	cout<<"xWgrip:"<<atof(argv[1])<<",yWgrip:"<<atof(argv[2])<<",zWgrip:"<<atof(argv[3])<<",yaw_WgripDegree:"<<atof(argv[4])<<endl;
	
	//run and print value from lab_invk, return it to driver_msg.
	driver_msg.destination=lab_invk(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]));
	
	cout<<"msg destination: \n"<<driver_msg<<endl;

	while(!ros::ok()){};

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;

	pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}

	ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
