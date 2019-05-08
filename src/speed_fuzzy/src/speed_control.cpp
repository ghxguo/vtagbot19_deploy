#include <ros/ros.h>
#include <std_msgs/Float64>
#include <novatel_msgs/NovatelVelocity.h>

float speed_fb = 0;
float setpoint = 0;
void speed_cb(const novatel_msgs::NovatelVelocity::ConstPtr& vel)
{
  speed_fb = vel.horizontal_speed;
}
void setpoint_cb(const std_msgs::Float64::ConstPtr& setpoint)
{
  setpoint = setpoint.data;
}
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "speed_fuzzy");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher pedal_pub = n.advertise<std_msgs::Float64>("pedal_effort", 1000);
  ros::Subscriber speed_sub = n.subscribe("bestvel", 1000, speed_cb);
  ros::Subscriber setpoint_sub = n.subscribe("speed_setpoint", 1000, setpoint_cb);
  std_msgs::Float64 effort;
  while(ros::ok())
  {
    float error = speed_set - speed_feedback
if error < 0 - speed_set*byPassBandWidth:
    msg.pedal_percent = byPassPedalPosMin
elif error > speed_set*byPassBandWidth:
    msg.pedal_percent = byPassPedalPosMax
else:
msg.pedal_percent = (byPassPedalPosMax + byPassPedalPosMin)/2
    pedal_pub.publish(effort);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
