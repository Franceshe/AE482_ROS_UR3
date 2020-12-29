/********************LAB 3.cpp*************
	only contains main function.
	The forward kinematics function is implemented in lab3func.cpp
*/

#include "lab3pkg/lab3.h"

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
	ros::init(argc, argv, "lab3node");
	ros::NodeHandle nh;
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	
	ece470_ur3_driver::command driver_msg;

	
	/**command line parse:
	 * for detail, google "main command line arguments c"
	 * command need to be length of 7 : lab3node theta1 theta2 theta3 theta4 theta5 theta6"
	 * Thetas in Degrees
	 */
	cout<<"argc = "<<argc<<endl;
	if(argc!=7){
		cout<<"invalid number of input"<<endl;
		cout<<"command should be entered in Degrees with format: rosrun lab3pkg lab3node theta1 theta2 theta3 theta4 theta5 theta6"<<endl;
		return 0;
	}
	cout<<"theta1:"<<atof(argv[1])<<",theta2:"<<atof(argv[2])<<",theta3:"<<atof(argv[3])<<",theta4:"<<atof(argv[4])<<",theta5:"<<atof(argv[5])<<",theta6:"<<atof(argv[6])<<endl;
	//~ float f = atof(sf); Function to turn string to float number.
	//assign return value from lab_angles to destination
	//lab_fk is passed angles in radians and calculates and prints forward kinematics.
	driver_msg.destination=lab_fk(atof(argv[1])*PI/180.0,atof(argv[2])*PI/180.0,atof(argv[3])*PI/180.0,atof(argv[4])*PI/180.0,atof(argv[5])*PI/180.0,atof(argv[6])*PI/180.0);
	cout<<"driver msg destination: \n"<<driver_msg<<endl;

	//Main Loop
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
