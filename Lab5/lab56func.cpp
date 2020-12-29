#include "lab56pkg/lab56.h"
#include "lab4pkg/lab4.h"
// #include "lab2pkg/lab2.h"
// #include "lab4pkg/lab4.h"
#include <algorithm>
#include <vector>

extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

bool state = 0;

float SuctionValue = 0.0;

bool leftclickdone = 1;
bool rightclickdone = 1;

bool get_block = 1;

vector<int> *ptr_to_obj_labels;
vector<int> *ptr_to_obj_centroids_x;
vector<int> *ptr_to_obj_centroids_y;

double O_r = 480*0.5;
double O_c = 640*0.5;
double beta = 784.57;
// double theta = PI + atan2((307-293), (408-73));
// double theta = PI + atan2((313-330), (303 - 219));
// double theta = 3.3052;
double theta = 3.4052;
// double theta = PI + atan2((330 - 313), (219 - 303));
// double theta = PI + atan2((219 - 303), (330 - 313));
// double theta = PI + atan2((303 - 219), (313-330));
// double theta = PI + atan2((104-214), (348-328));
double hypotenuse = 1.0;
// double Tx = (329.0-O_r)/beta;
// double Ty = (440.0-O_c)/beta;
double Tx = (440.0-O_r)/beta;
double Ty = (329.0-O_c)/beta;
// double Tx = (310)/beta;
// double Ty = (4)/beta;
// double Tx = (329.0)/beta;
// double Ty = (440.0)/beta;
double z_w =  0.057;

// double home[]={140.29*PI/180,-96.79*PI/180, 107.04*PI/180, -99.21*PI/180, -89.58*PI/180, 20.04*PI/180};
// //vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
// std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));
//

using namespace std;

/*****************************************************
* Functions in class:
* **************************************************/
/**
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
// std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
// {
// 	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end;
// 	double xgrip,ygrip,zgrip;
// 	double gripper_len;
// 	double theta_yaw;
// 	double a1,a2,a3,a4,a5,a6;
// 	double d1,d2,d3,d4,d5,d6;
//
// 	a1 = 0;
// 	d1 = 0.152;
// 	a2 = 0.244;
// 	d2 = 0.120;
// 	a3 = 0.213;
// 	d3 = -0.093;
// 	a4 = 0;
// 	d4 = 0.083;
// 	a5 = 0;
// 	d5 = 0.083;
// 	a6 = 0.0535;
// 	d6 = (0.082+0.056);
//
// 	xgrip = xWgrip;
// 	ygrip = yWgrip;
// 	zgrip = zWgrip;
//
// 	xgrip += 0.149;
// 	ygrip -= 0.149;
// 	zgrip -= 0.0193;
//
// 	gripper_len = 0.0535;
// 	theta_yaw = PI*((yaw_WgripDegree-90)/180);
// 	cout<<"theta_yaw: "<< theta_yaw<<endl;
//
// 	xcen = xgrip - (gripper_len * cos(theta_yaw - PI/2));
// 	ycen = ygrip + (gripper_len * sin(theta_yaw - PI/2));
// 	zcen = zgrip;
//
//
// 	//Calculate theta_1
// 	double theta_x = atan2(ycen, xcen);
// 	double q = (0.120 - 0.093) + 0.083;
// 	double d = sqrt((xcen*xcen) + (ycen*ycen));
// 	double theta_y = asin(q / d);
//
// 	//Calculate theta_5, theta_6
// 	theta1 = theta_x - theta_y;
// 	theta6 = theta1 - theta_yaw;
// 	theta5 = -PI/2;
//
// 	//# get x,y,z (end)
// 	double p1_0[] = {xcen, ycen};
// 	double theta = PI/2 - theta1;
// 	double R1_0[2][2];
// 	R1_0[0][0] = cos(theta);
// 	R1_0[0][1] = sin(theta);
// 	R1_0[1][0] = -sin(theta);
// 	R1_0[1][1] = cos(theta);
//
// 	double p2_1[2];
// 	p2_1[0] = 0.11;
// 	p2_1[1] = -0.083;
//
// 	double temp[2] = {R1_0[0][0] * p2_1[0] + R1_0[0][1] * p2_1[1],
// 	R1_0[1][0] * p2_1[0] + 	R1_0[1][1] *  p2_1[1] };
//
// 	x3end = p1_0[0] + temp[0];
// 	y3end = p1_0[1] + temp[1];
// 	z3end = zgrip + 0.052 + 0.082; //0.082 in manual, 0.052 by hand
// 	                               // it's from the end to the end of gripper
//
//
//     cout << x3end << '\n';
// 	cout << y3end << '\n';
// 	cout << z3end << '\n';
//
// 	cout << xcen << '\n';
// 	cout << ycen << '\n';
// 	cout << zcen << '\n';
//
// 	//Calculate theta_3
// 	double a = sqrt((x3end*x3end) + (y3end*y3end) + ((z3end-d1)*(z3end-d1)));
// 	theta3 = PI - acos(((a*a) - (a2*a2) - (a3*a3)) / (-2 * a2 * a3));
//
// 	//Calculate theta_2
// 	double b = sqrt((x3end*x3end) + (y3end*y3end));
// 	double gamma = atan2((z3end - d1), b);
// 	// double gamma = acos(b/a);
// 	cout << gamma << endl;
// 	//aplha that was just used for solving theta 4 can be used here too
// 	double alpha = asin((a3 * sin(PI - theta3))/a);
// 	//theta2 = -(acos( ((a2*a2) + (a*a) - (a3*a3)) / 2*a2*a )) + gamma;
// 	theta2 = -1*(gamma + alpha);
//
// 	//Calculate theta_4
// 	//Not changed
// 	double beta = PI - (PI - theta3) - alpha;
// 	theta4 = -(PI - (PI/2 - beta) - gamma);
//
// 	//The issues were maybe with (xcen, ycen), theta_yaw conversion,
// 	//but most likely with theta2 calculation
//
// 	//Input:  x = 0.2, y = 0.1, z=0.2, theta_yaw = 45 degrees
// 	//Outout: x = 0.211, y = 0.093, z = 0.174, theta_yaw = 43 degrees
//
// 	//Input:  x = 0.2, y = -0.1, z=0.2, theta_yaw = -45 degrees
// 	//Outout: x = 0.197, y = -0.075, z = 0.175, theta_yaw = -43 degrees
//
//
// 	// View values
// 	//use cout
// 	cout<<"theta1: "<< theta1<<endl;
// 	cout<<"theta2: "<< theta2<<endl;
// 	cout<<"theta3: "<< theta3<<endl;
// 	cout<<"theta4: "<< theta4<<endl;
// 	cout<<"theta5: "<< theta5<<endl;
// 	cout<<"theta6: "<< theta6<<endl;
//
// 	// check that your values are good BEFORE sending commands to UR3
// 	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
// 	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
// }
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
// /*Description:  Moves arm from 'current' position to destination
// * Inputs:       dest - destination to move the arm to
// *               duration - duration of the move (in seconds??)
// * Output:       0 if success
// */
// int move_arm(ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
// {
//   int error = 0;
//   ece470_ur3_driver::command driver_msg;
//   ROS_INFO("Moving arm");
//   driver_msg.destination=dest;
//   driver_msg.duration=duration;     // make sure duration is >0, we can develop some sort of
//                                     // error handler
//   pub_command.publish(driver_msg);  // publish command, but note that is possible that
//                                     // the subscriber will not receive this message.
//   int spincount = 0;
//   while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
//       ros::spinOnce();  // Allow other ROS functionallity to run
//       loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
//       if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
//           pub_command.publish(driver_msg);
//           //ROS_INFO("Just Published again driver_msg");
//           spincount = 0;
//       }
//       spincount++;  // keep track of loop countm
//   }
//
//   ROS_INFO("Waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
//   while(!isReady)
//   {
//       ros::spinOnce();
//       loop_rate.sleep();
//   }
//   return error;
// }

//constructor(don't modify)


ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1,
    	&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);
    pub_command=nh_.advertise<ece470_ur3_driver::command>("ur3/command",10);
    sub_position=nh_.subscribe("ur3/position",1,&ImageConverter::position_callback,this);

	sub_io_states=nh_.subscribe("ur_driver/io_states",1,&ImageConverter::suction_callback,this);

	srv_SetIO = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");


    driver_msg.destination=lab_invk(0,0,0.2,-90);

	//publish the point to the robot
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
										  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");

}

//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

void ImageConverter::suction_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	SuctionValue = msg->analog_in_states[0].state;

    if (SuctionValue > 2.0) {
        get_block = 0;
    }
    else {
        get_block = 1;
    }
}


//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // create an gray scale version of image
    Mat gray_image;
	cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );
    // convert to black and white img, then associate objects:

	Mat bw_image;
	// adaptiveThreshold(gray_image,bw_image,255,0,0,151,5);
    bw_image = thresholdImage(gray_image);
	//adaptiveThreshold(scr,dst,MAXVALUE,adaptiveMethod,thresholdType,blocksize,C);
	//adaptiveMethod = 0, ADAPTIVE_THRESH_MEAN_C
	//thresholdType = 0, BINARY
	//blocksize
	//C constant subtracted from tz.


// FUNCTION you will be completing
    Mat associate_image = associateObjects(bw_image); // find associated objects

    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
		int   totalpixels;
		Mat bw_img  = gray_img.clone(); // copy input image to a new image
		totalpixels	  = gray_img.rows*gray_img.cols;			// total number of pixels in image
		uchar graylevel; // use this variable to read the value of a pixel

		int zt=0; // threshold grayscale value



		zt = 100;  // you will be finding this automatically


		//std::cout<<zt<<std::endl;
		// threshold the image
		for(int i=0; i<totalpixels; i++)
		{
			graylevel = gray_img.data[i];
			if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
			else             bw_img.data[i]= 0; // set rgb to 0   (black)
		}
	return bw_img;
}
/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here
Mat ImageConverter::associateObjects(Mat bw_img)
{
	//initiallize the variables you will use
	int height,width; // number of rows and colums of image
	int red, green, blue; //used to assign color of each objects
	uchar pixel; //used to read pixel value of input image
	height = bw_img.rows;
	width = bw_img.cols;
	int num = 0;

    // O_r = 1/2*height;
    // O_c = 1/2*width;

	// initialize an array of labels, assigning a label number to each pixel in the image
	// this create a 2 dimensional array pixellabel[row][col]
	int ** pixellabel = new int*[height];
	for (int i=0;i<height;i++) {
		pixellabel[i] = new int[width];
	}

	int label_num = 1;
    int label[height*width]; // label array
    int noise[height*width]; // noise cancellation array
    int *equiv[height*width]; // label array pointers

    //Equiv Setup
    for (size_t i = 0; i < height*width; i++) {
        equiv[i] = &label[i];
        noise[i] = 0; // wow, just wow
    }

    for(int row=0; row<height; row++)
    {
    	for(int col=0; col<width; col++)
    	{
            if (bw_img.data[row*width + col] == 255)
    		    pixellabel[row][col] = -1;//white
            else
                pixellabel[row][col] = 0;//black
    	}
    }

    // // creating a demo image of colored lines
    // for(int row=0; row<height; row++)
    // {
    //     for(int col=0; col<width; col++)
    //     {
    //         pixellabel[row][col] = num;
    //     }
    //     num++;
    //     if (num == 10) {
    //         num = 0;
    //     }
    // }

    int Pixel, Left, Above, min_n, max_n;

    //First raster scan
    for(int row=0; row<height; row++)
    {
    	for(int col=0; col<width; col++)
    	{

            Pixel = pixellabel[row][col];
            if(col == 0)
                Left = -1;
            else
                Left = pixellabel[row][col-1];
            if(row == 0)
                Above = -1;
            else
                Above = pixellabel[row-1][col];

            if(Pixel != -1){
                if(Left == -1 && Above == -1){
                    pixellabel[row][col] = label_num;
                    label[label_num] = label_num;
                    label_num++;
                }
                if(Left != -1 && Above == -1)
                    pixellabel[row][col] = Left;
                if(Left == -1 && Above != -1)
                    pixellabel[row][col] = Above;

                if(Left != -1 && Above != -1){
                    int smallLabel = min(*equiv[Left], *equiv[Above]);
                    //*equiv[Left]>*equiv[Above] ? *equiv[Above]: *equiv[Left];
                    // pixellabel[row][col] = Left>Above ? Above:Left;
                    //int min = *equiv[Left] == smallLabel ? Left: Above;
                    //int max = *equiv[Left] == smallLabel ? Above: Left;
                    if(smallLabel == *equiv[Left]) {
                        min_n = Left;
                        max_n = Above;
                    }
                    else {
                        min_n = Above;
                        max_n = Left;
                    }
                    pixellabel[row][col] = smallLabel;

                    *equiv[max_n] = *equiv[min_n];
                    equiv[max_n] = equiv[min_n];
                }
            }
    	}
    }

    //Second raster scan
    for(int row=0; row<height; row++)
    {
    	for(int col=0; col<width; col++)
    	{
            Pixel = pixellabel[row][col];
            if(Pixel != -1){
                pixellabel[row][col] = *equiv[Pixel];
                noise[Pixel]++; // noise cancellation
            }
    	}
    }


    // noise boundaries
    // the window should only display the blocks
    int lower_boundary = 450;
    int upper_boundary = 1000;
    int difference = upper_boundary - lower_boundary;

    //used to keep track of the amount of objects
    vector<int> obj_labels;
    // ptr_to_obj_labels = new vector<int> obj_labels;
    pair<int, int> coordinate_val[height*width];
    // ptr_to_obj_centroids = new pair<int, int> coordinate_val[height*width];
    for (size_t i = 0; i < height*width; i++)
        coordinate_val[i].first = coordinate_val[i].second = 0;

	// assign UNIQUE color to each object
	Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
	Vec3b color;
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
            if((noise[pixellabel[row][col]] < lower_boundary) || noise[pixellabel[row][col]] > upper_boundary)
                pixellabel[row][col] = -1;

            //Adds valid objects to the vector
            if(pixellabel[row][col] != -1){

                coordinate_val[pixellabel[row][col]].first += col;
                coordinate_val[pixellabel[row][col]].second += row;

                bool vec_flag = true;
                for (size_t i = 0; i < obj_labels.size(); i++) {
                    if(obj_labels[i] == pixellabel[row][col]){
                        vec_flag = false;
                        break;
                    }
                }
                if(vec_flag){
                    obj_labels.push_back(pixellabel[row][col]);
                }
            }

            int vec_pos = -1;

            for (size_t i = 0; i < obj_labels.size(); i++) {
                if(obj_labels[i] == pixellabel[row][col]){
                    vec_pos = i;
                    break;
                }
            }

            switch (vec_pos)
			{

				case -1:
					red    = 255; // you can change color of each objects here
					green = 255;
					blue   = 255;
					break;
                case 0:
                    red    = 0; // you can change color of each objects here
                    green = 0;
                    blue   = 255;
                    break;
				case 1:
					red    = 255; // you can change color of each objects here
					green  = 0;
					blue   = 0;
					break;
				case 2:
					red    = 0;
					green  = 255;
					blue   = 0;
					break;
				case 3:
                    red    = 0; // you can change color of each objects here
                    green = 0;
                    blue   = 204;
					break;
				case 4:
					red    = 255;
					green  = 255;
					blue   = 0;
					break;
				case 5:
					red    = 255;
					green  = 0;
					blue   = 255;
					break;
				case 6:
					red    = 0;
					green  = 255;
					blue   = 255;
					break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                 	break;
				default:
					red    = 0;
					green = 0;
					blue   = 0;
					break;
			}

			color[0] = blue;
			color[1] = green;
			color[2] = red;
			associate_img.at<Vec3b>(Point(col,row)) = color;
		}
	}
    // vector<pair<int, int> > obj_centroids;
    vector<int> obj_centroids_x;
    vector<int> obj_centroids_y;
    for (size_t i = 0; i < obj_labels.size(); i++) {
        int label_val = obj_labels[i];
        pair<int, int> temp;
        temp.first = coordinate_val[label_val].first;
        temp.second = coordinate_val[label_val].second;
        temp.first /= noise[label_val];
        temp.second /= noise[label_val];
        // temp.first--;
        // temp.second--;
        obj_centroids_x.push_back(temp.first);
        obj_centroids_y.push_back(temp.second);
    }

    // double base = 1.0;
    // double hypotenuse = 1.0;

    for (size_t i = 0; i < obj_centroids_y.size() && i < 2; i++) {
        // if(i==0){
            // base = obj_centroids_y[i];
        // }
        // else if(i == 1){
        if(i == 1){
            // base = abs(base - obj_centroids_y[i]);
            hypotenuse = sqrt(pow(obj_centroids_x[1] - obj_centroids_x[0],2) + pow(obj_centroids_y[1] - obj_centroids_y[0],2));
        }
    }

    // theta = acos(base/hypotenuse);


    // Vec3b color;
    // color[0] = 255;
    // color[1] = 255;
    // color[2] = 255;
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    // std::cout << "There are "<< obj_labels.size() << "objects" << std::endl;
    for (int i = 0; i < obj_labels.size(); i++) {
        // std::cout << "Object " << i << ": " << '\n';
        // std::cout << "x centroid :" << obj_centroids_x[i] << '\n';
        // std::cout << "y centroid :" << obj_centroids_y[i] << '\n';
        int height = 480;
        int width = 640;

        for (int j = 0; j < 4; j++) {
            int x = obj_centroids_x[i];
            int y = obj_centroids_y[i];

            if(x+j < width && x+j>0 && x+j < width){
                // cout << "1" <<endl;
                associate_img.at<Vec3b>(Point(x+j,y)) = color;
            }

            if(x-j > 0 && x-j > 0 && x-j < width){
                // cout << "2" <<endl;
                associate_img.at<Vec3b>(Point(x-j,y)) = color;
            }

            if(y+j < height && y+j > 0 && y+j < height){
                // cout << "3" <<endl;
                associate_img.at<Vec3b>(Point(x,y+j)) = color;
            }

            if(y-j > 0 && y-j > 0 && y+j < height){
                // cout << "4" <<endl;
                // cout << (y-j) <<endl;
                associate_img.at<Vec3b>(Point(x,y-j)) = color;
            }
        // line((640/2)-20, screenHeight/2, (640/2)+20, screenHeight/2);

        }
            // int x = obj_centroids_x[i];
            // int y = obj_centroids_y[i];
            // associate_img.at<Vec3b>(Point(x,y)) = color;

    }
    // std::cout << "x: " << width << '\n';
    // std::cout << "y: " << height << '\n';



    ptr_to_obj_labels = new vector<int>(obj_labels.size());
    ptr_to_obj_centroids_x = new vector<int>(obj_centroids_x.size());
    ptr_to_obj_centroids_y = new vector<int>(obj_centroids_y.size());

    ptr_to_obj_labels->swap(obj_labels);
    ptr_to_obj_centroids_x->swap(obj_centroids_x);
    ptr_to_obj_centroids_y->swap(obj_centroids_y);
    // for (size_t i = 0; i < count; i++) {
    //     /* code */
    // }
    // std::cout << "reached" << height << '\n';
	return associate_img;
}
// void move_arm(double x, double y, double z){
//     // put your left click code here
//     driver_msg.destination=lab_invk(x_w, y_w, z_w,-90);
//
//     //publish the point to the robot
//     ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
//     int spincount = 0;
//     driver_msg.duration = 3.0;
//     pub_command.publish(driver_msg);  // publish command, but note that is possible that
//                                           // the subscriber will not receive this message.
//     spincount = 0;
//     while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
//         ros::spinOnce();  // Allow other ROS functionallity to run
//         loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
//         if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
//             pub_command.publish(driver_msg);
//             ROS_INFO_STREAM("Just Published again driver_msg");
//             spincount = 0;
//         }
//         spincount++;  // keep track of loop count
//     }
//     ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
//
//     while(!isReady)
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     ROS_INFO_STREAM("Ready for new point");
//
// }
/*****************************************************
	*Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h"
void onMouse(int event, int x, int y, int flags, void* userdata)
{
		ic_ptr->onClick(event,x,y,flags,userdata);
}
void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command

	// For use with Lab 6
	// If the robot is holding a block, place it at the designated row and column.
	if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
	{
		if (leftclickdone == 1) {
			leftclickdone = 0;  // code started


            if(get_block){
                // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
                suction_off(srv_SetIO, srv, loop_rate);
                get_block = 0;
            // put your right click code here
            //Store the parameters transfer btw Camera frame and world frame
            double r = y;
            double c = x;
            double x_c = (y-O_r)/beta;
            double y_c =(x-O_c)/beta;
            double x_w = x_c*cos(theta) + y_c*sin(theta) - Tx*cos(theta) - Ty*sin(theta);
            double y_w = -1 * x_c*sin(theta) + y_c*cos(theta) + Tx*sin(theta) -Ty*cos(theta);
            x_w += 0.02;
            y_w -= 0.055;

			ROS_INFO_STREAM("left click:  (" << x_w << ", " << y_w << ")");  //the point you clicked


            //Move to home position
            // move_arm(0, 0, 0.2);
            driver_msg.destination=lab_invk(0, 0, 0.2,-90);

            //publish the point to the robot
            // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            int spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");


            //Move to desired position
            // move_arm(x_w, y_w, z_w);
            driver_msg.destination=lab_invk(x_w, y_w, z_w,-90);

            //publish the point to the robot
            // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");


            suction_on(srv_SetIO, srv, loop_rate);
            //Move to home position
            // move_arm(0, 0, 0.2);
            driver_msg.destination=lab_invk(0, 0, 0.2,-90);

            //publish the point to the robot
            // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");

            }
            leftclickdone = 1; // code finished
        } else {
            ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click");
        }
	}
	else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
	{
		if (rightclickdone == 1) {  // if previous right click not finished ignore
			rightclickdone = 0;  // starting code

            if(!get_block){
                get_block = 1;
            // put your right click code here
            //Store the parameters transfer btw Camera frame and world frame
            double r = y;
            double c = x;
            double x_c = (y-O_r)/beta;
            double y_c =(x-O_c)/beta;
            double x_w = x_c*cos(theta) + y_c*sin(theta) - Tx*cos(theta) - Ty*sin(theta);
            double y_w = -1 * x_c*sin(theta) + y_c*cos(theta) + Tx*sin(theta) -Ty*cos(theta);
            x_w += 0.015;
            y_w -= 0.035;

            ROS_INFO_STREAM("right click:  (" << x_w << ", " << y_w << ")");  //the point you clicked

            //Move to home position
            // move_arm(0, 0, 0.2);
            driver_msg.destination=lab_invk(0, 0, 0.2,-90);

            //publish the point to the robot
            ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            int spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");

            //Move to desired position
            // move_arm(x_w, y_w, z_w);
            driver_msg.destination=lab_invk(x_w, y_w, z_w,-90);

            //publish the point to the robot
            // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");





            suction_off(srv_SetIO, srv, loop_rate);
            //Move to home position
            // move_arm(0, 0, 0.2);
            driver_msg.destination=lab_invk(0, 0, 0.2,-90);

            //publish the point to the robot
            // ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
            spincount = 0;
            driver_msg.duration = 3.0;
            pub_command.publish(driver_msg);  // publish command, but note that is possible that
                                                  // the subscriber will not receive this message.
            spincount = 0;
            while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
                ros::spinOnce();  // Allow other ROS functionallity to run
                loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
                if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO_STREAM("Just Published again driver_msg");
                    spincount = 0;
                }
                spincount++;  // keep track of loop count
            }
            ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

            while(!isReady)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO_STREAM("Ready for new point");


            // //Move to home position
            // move_arm(0, 0, 0.2);
            // //Move to desired position
            // move_arm(x_w, y_w, z_w);
            // suction_off(srv_SetIO, srv, loop_rate);
            // //Move to home position
            // move_arm(0, 0, 0.2);
            }

			rightclickdone = 1; // code finished
		} else {
			ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click");
		}
	}
}
