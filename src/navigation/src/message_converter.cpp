#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>
#include <sstream>


void move_base_cmd_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  static ros::NodeHandle n;
  static ros::Publisher controller_pub = n.advertise<std_msgs::Int16MultiArray>("/ecat_ms_out", 1000);

  geometry_msgs::Twist newTwist(*msg);
  std_msgs::Int16MultiArray int16msg;
  int16msg.data.clear();
  int16msg.data.push_back( (10 * msg->angular.z) );
  int16msg.data.push_back( (10 * msg->linear.x) );
  controller_pub.publish(int16msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptor");
  ros::NodeHandle n;

  ros::Subscriber move_base_sub = n.subscribe("cmd_vel", 100, move_base_cmd_Callback);
  ros::spin();

  return 0;
}
