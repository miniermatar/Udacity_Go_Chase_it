#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <string>
// Ros global client
ros::ServiceClient client;

void drive_robot (float lin_x, float ang_z)
{
	
	ROS_INFO_STREAM ("Chasing the white ball");
	//Send moving command to the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x=lin_x;
	srv.request.angular_z=ang_z;
	
	//call DriveTotarget and rport error if it fails
	if (!client.call(srv))
		ROS_ERROR ("Failed to call service ball_chaser");	
	
}


void process_image_callback (const sensor_msgs::Image img)

{
	/* Counting white pixel detected on the screen and their position to calculate the average ball location on the image. The height and width of the camera is 800 x 800 pixel; the encoding is rgb8 so each pixel has 3 bytes making the step 2400 (this information was identified by printing those parameters on the screen using ROS_INFO_STREAM as below.
	
		std::string feedback = "Width: "+ std::to_string(img.width) + "Height " + std::to_string(img.height) + " Step: "+ std::to_string(img.step) + " Encoding "+img.encoding;
		ROS_INFO_STREAM(feedback);
	*/
	
	int white_pixel=255;
	int white_count=0;
	int white_col_sum=0;

	for (int i=0;i<(img.width*img.step);i+=3)
	{
		if (img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel)
		{
			white_count+=1;
			white_col_sum+=(i % img.step)/3; //dividing by 3 since each pixel has 3 bytes
		}
	}
	
	if (white_count==0)
		drive_robot(0.0,0.0);
	else
	{
		int white_mean_loc=white_col_sum/white_count;
		if (white_mean_loc<img.width/3) // dividing by 3 to account for 1/3 of the screen
			drive_robot(0.0,-0.5);
		else if (white_mean_loc>img.width*2/3) //multiplying by 2/3 to account for the last 2/3 of the screen
			drive_robot(0.0,0.5);
		else
			drive_robot(0.5,0.0);
	}	
	
	

	

}



int main (int argc, char** argv)
{
	//initialize process image
	ros::init(argc,argv,"process_image");
	ros::NodeHandle n;
	
	//Define client service capable of requesting services from command_robot
	client =n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
	//Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1=n.subscribe("/camera/rgb/image_raw",10,process_image_callback);
	//ros communication
	ros::spin();
	
	
	
	return 0;
}
