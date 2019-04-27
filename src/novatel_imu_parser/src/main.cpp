#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "novatel_imu_parser/parser.h"
#include "ros/ros.h"
#include <iostream>



typedef boost::chrono::steady_clock time_source;

void parserThread(ros::Rate rate, parser::Novatel_imu* imu)
{
  time_source::time_point last_time = time_source::now();
  while (ros::ok())
  {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;
    imu->subscribeToIMU();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "novatel_imu_parser");
  parser::Novatel_imu handle;
  boost::thread(boost::bind(parserThread, ros::Rate(50), &handle));
  ros::spin();
  return 0;
}
