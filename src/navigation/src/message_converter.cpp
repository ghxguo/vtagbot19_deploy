#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>
#include <sstream>

std_msgs::Int16MultiArray int16msg;

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
	int16msg.data[0] = msg->angular.z;

	std_msgs::Float64 speed;
	speed.data = msg->linear.x;
	speed_setpoint_pub.publish(speed);
}

void pedal_callback(const std_msgs::Float64::ConstPtr &msg)
{
	int16msg.data[1] = msg->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adaptor");
	int16msg.data.push_back(0);
	int16msg.data.push_back(0);
	ros::NodeHandle n;
	ros::Subscriber move_base_sub = n.subscribe("cmd_vel", 100, move_base_cmd_Callback);
	ros::Subscriber pedal_sub = n.subscribe("pedal_effort", 100, pedal_callback);
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
