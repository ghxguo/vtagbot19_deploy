#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>
#include <sstream>
#include <novatel_gps_msgs/NovatelVelocity.h>

std_msgs::Int16MultiArray int16msg;

//convert from novatel velocity msg to float64 for pid feedback
void vel_callback(const novatel_gps_msgs::NovatelVelocity::ConstPtr &msg)
{
	static ros::NodeHandle n;
	static ros::Publisher speed_feedback_pub = n.advertise<std_msgs::Float64>("/speed_feedback", 50);
	std_msgs::Float64 speed_feedback;
	speed_feedback.data = msg->horizontal_speed;
	speed_feedback_pub.publish(speed_feedback);
}

//convert twist velocity to flot64 as setpoint for pid, also add steer angle to multiarray
void move_base_cmd_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	// static ros::NodeHandle n;
	// static ros::Publisher controller_pub = n.advertise<std_msgs::Int16MultiArray>("/ecat_ms_out", 1000);
	// geometry_msgs::Twist newTwist(*msg);
	// std_msgs::Int16MultiArray int16msg;
	// int16msg.data.clear();
	// int16msg.data.push_back( (10 * msg->angular.z) );
	// int16msg.data.push_back( (10 * msg->linear.x) );
	// controller_pub.publish(int16msg);

	static ros::NodeHandle n;
	static auto speed_setpoint_pub = n.advertise<std_msgs::Float64>("speed_setpoint", 100);
	int16msg.data[0] = msg->angular.z / 0.4363 * 1000;
	int temp = 0 - msg->angular.z / 0.4363 * 1000;
	ROS_INFO("Temp: %d", temp);
	int16msg.data[0] = -int16msg.data[0];
	ROS_INFO("Data: %d", int16msg.data[0]);
	std_msgs::Float64 speed;
	speed.data = msg->linear.x;
	speed_setpoint_pub.publish(speed);
}

//set the pedal effort from pid to multy array
void pedal_callback(const std_msgs::Float64::ConstPtr &msg)
{
	int16msg.data[1] = msg->data * 10;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adaptor");
	int16msg.data.push_back(0);
	int16msg.data.push_back(0);
	ros::NodeHandle n;
	ros::Subscriber move_base_sub = n.subscribe("cmd_vel", 100, move_base_cmd_Callback);
	ros::Subscriber pedal_sub = n.subscribe("pedal_effort", 100, pedal_callback);
	auto vel_sub = n.subscribe("bestvel", 50, vel_callback);
	ros::Publisher controller_pub = n.advertise<std_msgs::Int16MultiArray>("/ecat_ms_out", 100);
	ros::spinOnce();
	ros::Rate loopRate(100);
	while (ros::ok())
	{
		controller_pub.publish(int16msg);
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
