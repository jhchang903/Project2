#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Define a global client that can request services
ros::ServiceClient client;

//This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){
	ROS_INFO_STREAM("Drive the robot toward the ball");

	//Call the /ball_chaser/command_robot service and pass the requested velocities to it to drive the robot

	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if (!client.call(srv))
		ROS_ERROR("Failed to call service /ball_chaser/command");
}

//This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img){
	int white_pixel = 255;
	int col=0, positive=0;
	//Loop through each pixel in the image and check if there is a bright white one
	//Localize the central of that ball
	for(int i=0; i<img.height * img.step; i=i+3){
		if (img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel){
			col = col + i%img.step;
			positive++;
		}
	}
	
	
	//Request a stop when there is no white ball seen by the camera
	//Depending on the white ball position, call the drive_robot function and pass velocities to it	
	if (positive == 0) {
		drive_robot(0.0, 0.0);
		ROS_INFO_STREAM("STOP");
	}
	else if (col/positive<img.step/3) {
		ROS_INFO_STREAM("Turn left");
		drive_robot(0.2, 0.5);
		
	}
	else if (col/positive>=img.step/3 && col/positive<img.step*2/3) {
		drive_robot(0.2, 0.0);
		ROS_INFO("Go straight");
	}
	else if (col/positive>=img.step*2/3) {
		drive_robot(0.2, -0.5);
		ROS_INFO("Turn right");
	}else{
		ROS_ERROR("ERR");
	}
}


int main(int argc, char** argv){
	//Initailze the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	//Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	//Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback); 

	//Handle ROS communication events
	ros::spin();

	return 0;
}
