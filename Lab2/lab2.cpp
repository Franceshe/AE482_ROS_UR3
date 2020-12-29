#include "lab2pkg/lab2.h"
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

// arrays defining Waypoints
// home array
double home[]={140.29*PI/180,-96.79*PI/180, 107.04*PI/180, -99.21*PI/180, -89.58*PI/180, 20.04*PI/180};

// base and altitude array, example: arrBA where B is a base number, A is altitude
// altitude with one block on the ground is 1, two blocks on the ground 2, and so on
double arr11[]={124.48*PI/180, -57.36*PI/180, 117.65*PI/180, -149.27*PI/180, -89.96*PI/180, 4.54*PI/180};
double arr12[]={124.49*PI/180, -64.25*PI/180, 116.27*PI/180, -141.01*PI/180, -89.92*PI/180, 4.51*PI/180};
double arr13[]={124.51*PI/180, -70.23*PI/180, 113.76*PI/180, -132.51*PI/180, -89.89*PI/180, 4.48*PI/180};

double arr21[]={133.74*PI/180, -62.85*PI/180, 129.66*PI/180, -155.77*PI/180, -89.84*PI/180, 13.82*PI/180};
double arr22[]={133.77*PI/180, -71.19*PI/180, 128.05*PI/180, -145.83*PI/180, -89.80*PI/180, 13.80*PI/180};
double arr23[]={133.78*PI/180, -78.39*PI/180, 125.16*PI/180, -135.73*PI/180, -89.77*PI/180, 13.77*PI/180};

double arr31[]={150.57*PI/180, -65.93*PI/180, 137.40*PI/180, -160.45*PI/180, -89.59*PI/180, 30.65*PI/180};
double arr32[]={150.59*PI/180, -75.92*PI/180, 135.54*PI/180, -148.61*PI/180, -89.54*PI/180, 30.63*PI/180};
double arr33[]={150.61*PI/180, -84.10*PI/180, 132.33*PI/180, -137.23*PI/180, -89.49*PI/180, 30.60*PI/180};


// array to define final velocity of point to point moves.  For now slow down to zero once
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));

std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));

std::vector<double> Q31 (arr31,arr31+sizeof(arr31) / sizeof(arr31[0]));
std::vector<double> Q32 (arr32,arr32+sizeof(arr32) / sizeof(arr32[0]));
std::vector<double> Q33 (arr33,arr33+sizeof(arr33) / sizeof(arr33[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q[3][3] = {
    {Q11, Q12, Q13},
    {Q21, Q22, Q23},
    {Q31, Q32, Q33}
};


// Global bool variables that are assigned in the callback associated when subscribed
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;
bool state = 0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
  isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
                        // and is ready to be told to go to another point if desired.
  pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
                        // ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}


// Whenever ur3/position publishes info this callback function is run.
void suction_callback(const ur_msgs::IOStates::ConstPtr& msg) {
// {   cout << "in call back!!!!" << endl;
    double temp = msg->analog_in_states[0].state;
    // cout << temp << endl;
    if (temp > 2.0) {
        state = 1;
    }
    else {
        state = 0;
    }
    // cout << state << endl;
}


/*Description:  Moves arm from 'current' position to destination
* Inputs:       dest - destination to move the arm to
*               duration - duration of the move (in seconds??)
* Output:       0 if success
*/
int move_arm(ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
  int error = 0;
  ece470_ur3_driver::command driver_msg;
  ROS_INFO("Moving arm");
  driver_msg.destination=dest;
  driver_msg.duration=duration;     // make sure duration is >0, we can develop some sort of
                                    // error handler
  pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                    // the subscriber will not receive this message.
  int spincount = 0;
  while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
      ros::spinOnce();  // Allow other ROS functionallity to run
      loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
      if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
          pub_command.publish(driver_msg);
          //ROS_INFO("Just Published again driver_msg");
          spincount = 0;
      }
      spincount++;  // keep track of loop countm
  }

  ROS_INFO("Waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
  while(!isReady)
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return error;
}

/*Description:  Turns the suction ON. Helper for move_block function
* Inputs:
* Output:       0 if success
*               Prints True if success, False if fail
*/
int suction_on(ros::ServiceClient srv_SetIO, ur_msgs::SetIO srv, ros::Rate loop_rate) {
  int error = 0;
  srv.request.fun = 1;
  srv.request.pin = 0;  //Digital Output 0
  srv.request.state = 1.0; //Set DO0 on
  if (srv_SetIO.call(srv)) {
    //   ROS_INFO("True: Switched Suction ON");
  } else {
      ROS_INFO("False");
  }
  // ROS_INFO("Waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
  while(!isReady)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return error;
}

/*Description:  Turns the suction OFF. Helper for move_block function
* Inputs:
* Output:       0 if success
*               Prints True if success, False if fail
*/
int suction_off(ros::ServiceClient srv_SetIO, ur_msgs::SetIO srv, ros::Rate loop_rate) {
  int error = 0;
  srv.request.fun = 1;
  srv.request.pin = 0; // Digital Output 0
  srv.request.state = 0.0; //Set DO0 off
  if (srv_SetIO.call(srv)) {
      ROS_INFO("True: Switched Suction OFF");
  } else {
      ROS_INFO("False");
  }
  ROS_INFO("Waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
  while(!isReady)
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return error;
}

/*Description:  Moves the block from the start location to end location,
*               taking start and end height into consideration
* Inputs:       dest - destination to move the arm to
*               duration - duration of the move (in seconds??)
* Output:       0 if success
* Note:         location and height variable defined from 0-2, where 0 is
*               first row/column or first altitude
*/
int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,
                int start_height,
                int end_loc,
                int end_height)
{
  int error = 0;
  move_arm(pub_command, loop_rate, QH, 1.0);
  move_arm(pub_command, loop_rate, Q[start_loc][start_height], 1.0);
  suction_on(srv_SetIO, srv, loop_rate);
  ros::spinOnce();
  loop_rate.sleep();
  move_arm(pub_command, loop_rate, QH, 1.0);
  // no block present case
  if (state == 0) {
      error = -1;
      suction_off(srv_SetIO, srv, loop_rate);

      return error;
  }

  move_arm(pub_command, loop_rate, Q[end_loc][end_height], 1.0);
  suction_off(srv_SetIO, srv, loop_rate);
  move_arm(pub_command, loop_rate, QH, 1.0);
  return error;
}

/*Description:  Function to solve towerOfHanoi problem by recursion
* Inputs:       n - number of disks
*               from_rod - position from which to move disk from
*               from_alt - altitide from which to move disk from suction_on(srv_SetIO, srv, loop_rate);
*               to_rod - position to which to move disk from
*               to_alt - altitide to which to move di
sk from
*               aux_rod - position of the helper position
*               aux_alt - altitide of the helper position
* Output:
* Note:         In order to solve this problem recursively, altitudes must be
*               constantly updated!!!
*               Reference: https://youtu.be/5_6nsViVM00
*/

int globalCheck = 0;
void towerOfHanoi(int n, int from_rod, int &from_alt,
                    int aux_rod, int &aux_alt,
                    int to_rod, int &to_alt,

                    ros::Publisher pub_command,
                    ros::Rate loop_rate, ros::ServiceClient srv_SetIO,
                    ur_msgs::SetIO srv) {
  if (globalCheck == 1) {
    return;
    }

  if (n==1) {
      printf("\n Move disk from rod %i to rod %i", from_rod, to_rod);
      to_alt++;
      int returnValue = move_block(pub_command, loop_rate, srv_SetIO, srv, from_rod, from_alt, to_rod, to_alt);
      if (returnValue == -1) {
          ROS_INFO("Block not present!");
          // halt machine
          globalCheck = 1;
        //   ros::shutdown();
          return;
      }
      from_alt--;
      return;
  }
  towerOfHanoi(n-1, from_rod, from_alt, to_rod, to_alt, aux_rod, aux_alt, pub_command, loop_rate, srv_SetIO, srv);
  towerOfHanoi(1, from_rod, from_alt, aux_rod, aux_alt, to_rod, to_alt, pub_command, loop_rate, srv_SetIO, srv);
  towerOfHanoi(n-1, aux_rod, aux_alt, from_rod, from_alt, to_rod, to_alt, pub_command, loop_rate, srv_SetIO, srv);
}

/*Description:  Main program to solve Tower of Hanoi problem
* Inputs:
* Output:
*/
int main(int argc, char **argv)
{
  //initialization & variable definition
  ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
  ros::NodeHandle nh;				//handler for this node.
  //initialized publisher ur3/command, buffer size of 10.
  ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
  // initialize subscriber to ur3/position and call function position_callback each time data is published
  ros::Subscriber sub_position=nh.subscribe("ur3/position", 1, position_callback);
  ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
  ur_msgs::SetIO srv;
  ece470_ur3_driver::command driver_msg;
  ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command

  // initialize subscriber to ur3_driver/io_states
  ros::Subscriber suction_state = nh.subscribe("ur_driver/io_states", 1, suction_callback);
  // Variables for user input
  int inputdone = 0;
  int startPosition;
  int endPosition;
  std::string inputStart;
  std::string inputEnd;
// while(1) {
    // suction_on(srv_SetIO, srv, loop_rate);
    // ros::spinOnce();
    // loop_rate.sleep();
// }
  while (!inputdone) {
    std::cout << "Enter start position <Either 0 1 or 2>";
    std::getline(std::cin, inputStart);
    std::cout << "You entered " << inputStart << "\n";

    if (inputStart == "0") {
      startPosition = 0;
    } else if (inputStart == "1") {
      startPosition = 1;
    } else if (inputStart == "2") {
      startPosition = 2;
    } else {
      std:cout << "Please just enter the character 0 1 or 2\n\n";
    }

    std::cout << "Enter end position <Either 0 1 or 2>";
    std::getline(std::cin, inputEnd);
    std::cout << "You entered " << inputEnd << "\n";
    if (inputEnd == "0") {
        endPosition = 0;
    } else if (inputEnd == "1") {
        endPosition = 1;
    } else if (inputEnd == "2") {
        endPosition = 2;
    } else {
        cout << "Please just enter the character 0 1 or 2\n\n";
    }
    inputdone = 1;
  }

  while(!ros::ok()){};	//check if ros is ready for operation

/* Note: Input from user is already set, use following variables:
*  startPosition - starting position entered by user
*  endPosition - ending position entered by user
*/

  // TowerOfHanoi problem variables
  int n = 3;   // number of disks
  int aux;
  // Set the altitude based on starting position
  if (startPosition == 0 && endPosition == 1) {
    int Alt0 = 2; int Alt1 = -1; int Alt2 = -1; aux = 2;
    towerOfHanoi(n, startPosition, Alt0, aux, Alt2, endPosition, Alt1,  pub_command, loop_rate, srv_SetIO, srv);
  }
  else if (startPosition == 0 && endPosition == 2) {
    int Alt0 = 2; int Alt1 = -1; int Alt2 = -1; aux = 1;
    towerOfHanoi(n, startPosition, Alt0, aux, Alt1, endPosition, Alt2,  pub_command, loop_rate, srv_SetIO, srv);
  }
  else if (startPosition == 1 && endPosition == 0) {
    int Alt0 = -1; int Alt1 = 2; int Alt2 = -1; aux = 2;
    towerOfHanoi(n, startPosition, Alt1, aux, Alt2, endPosition, Alt0,  pub_command, loop_rate, srv_SetIO, srv);
  }
  else if (startPosition == 1 && endPosition == 2) {
    int Alt0 = -1; int Alt1 = 2; int Alt2 = -1; aux = 0;
    towerOfHanoi(n, startPosition, Alt1, aux, Alt0, endPosition, Alt2,  pub_command, loop_rate, srv_SetIO, srv);
  }
  else if (startPosition == 2 && endPosition == 0) {
    int Alt0 = -1; int Alt1 = -1; int Alt2 = 2; aux = 1;
    towerOfHanoi(n, startPosition, Alt2, aux, Alt1, endPosition, Alt0,  pub_command, loop_rate, srv_SetIO, srv);
  }
  else if (startPosition == 2 && endPosition == 1) {
    int Alt0 = -1; int Alt1 = -1; int Alt2 = 2; aux = 0;
    towerOfHanoi(n, startPosition, Alt2, aux, Alt0, endPosition, Alt1,  pub_command, loop_rate, srv_SetIO, srv);
  }

  return 0;
}
