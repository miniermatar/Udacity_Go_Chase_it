#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

//ROS Publisher for motor commands
ros::Publisher motor_command_publisher;


bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
	geometry_msgs::Twist motor_command;
	motor_command.linear.x=req.linear_x;
	motor_command.angular.z=req.angular_z;
	motor_command_publisher.publish(motor_command);
	
	//Wait 0.1 sec for motion to start
	ros::Duration(0.1).sleep();
	
	//Response message
	res.msg_feedback = "Requested wheel velocities are: linear_x = "+ std::to_string(req.linear_x) + " angular_z = "+ std::to_string(req.angular_z);
	ROS_INFO_STREAM(res.msg_feedback);
	
	return true;
}

int main(int argc, char** argv)
{
	//ROS node initialization
	ros::init (argc, argv, "drive_bot");
	
	//ROS nodehandle
	ros::NodeHandle n;
	
	//ROS publihser: geometry_msgs::Twist on the ros actuation topic /cmd_vel with a queue of 10
	motor_command_publisher=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	// Define service
	ros::ServiceServer service=n.advertiseService("/ball_chaser/command_robot",handle_drive_request);
	ROS_INFO("Ready to send commands");
	
	//Ros communication
	ros::spin();
	
	return 0;
}
