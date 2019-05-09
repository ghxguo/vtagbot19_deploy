#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <novatel_gps_msgs/NovatelVelocity.h>

float speed_fb = 0;
float  speed_setpoint= 0;
double bw = 0;
int pedal_acc = 0;
int pedal_dec = 0;
int pedal_const = 0;
void speed_cb(const novatel_gps_msgs::NovatelVelocity::ConstPtr& vel)
{
  speed_fb = vel->horizontal_speed;
}
void speed_setpoint_cb(const std_msgs::Float64::ConstPtr& setpoint)
{
  speed_setpoint = setpoint->data;
}
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "speed_fuzzy");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher pedal_pub = n.advertise<std_msgs::Float64>("pedal_effort", 1000);
  ros::Subscriber speed_sub = n.subscribe("bestvel", 1000, speed_cb);
  ros::Subscriber speed_setpoint_sub = n.subscribe("speed_setpoint", 1000, speed_setpoint_cb);
  std_msgs::Float64 effort;
  while(ros::ok())
  {
    n.param<double>("/speed_control/bandwidth", bw, 0.1);
    n.param("/speed_control/pedal_high", pedal_acc, 0);
    n.param("/speed_control/pedal_low", pedal_dec, 0);
    n.param("/speed_control/pedal_const", pedal_const, 0);
    // ROS_INFO("bandwidth: %f", bw);
    float error = speed_setpoint - speed_fb;
    // ROS_INFO("error: %f,", error);
    if (error < 0 - speed_setpoint*bw)
    {
      effort.data = pedal_dec;
    }
    else if (error > speed_setpoint*bw)
    {
      effort.data = pedal_acc;
    }
    else
    {
      effort.data = pedal_const;
    }
    if(speed_setpoint < 0.1)
    {
      effort.data = -500;
    }
    pedal_pub.publish(effort);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
