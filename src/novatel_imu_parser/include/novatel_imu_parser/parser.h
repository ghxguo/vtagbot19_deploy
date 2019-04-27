#ifndef PARSER_H
#define PARSER_H

#include "boost/thread.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "novatel_gps_msgs/NovatelCorrectedImuData.h"
#include "novatel_gps_msgs/Inspva.h"
#include "novatel_gps_msgs/Insstdev.h"
#include "tf/tf.h"

namespace parser
{
  class Novatel_imu
  {
  public:
    Novatel_imu();
    void subscribeToIMU();
  private:
    void corrimudataCallback(const novatel_gps_msgs::NovatelCorrectedImuData msg);
    void inspvaCallback(const novatel_gps_msgs::Inspva msg);
    void insstdevCallback(const novatel_gps_msgs::Insstdev msg);

     ros::NodeHandle nh_;
     ros::Subscriber corrimudata_sub_;
     ros::Subscriber inspva_sub_;
     ros::Subscriber insstdev_sub_;
     ros::Publisher compiled_imu_pub;

   struct InspvaData
   {
     float pitch;
     float roll;
     float azimuth;
     float x_vel;
     float y_vel;
     float z_vel;

     InspvaData() : pitch(0), roll(0), azimuth(0), x_vel(0), y_vel(0), z_vel(0)
     {
     }
   }
   inspva_data_;
   novatel_gps_msgs::Inspva inspva_data_msg_;
   boost::mutex inspva_data_msg_mutex_;

   struct Insstdev
   {
     float roll_dev;
     float pitch_dev;
     float azimuth_dev;

     Insstdev() : roll_dev(0), pitch_dev(0), azimuth_dev(0)
     {
     }
   }
   insstdev_data_;
   novatel_gps_msgs::Insstdev insstdev_data_msg_;
   boost::mutex insstdev_data_msg_mutex_;


   struct Corrimudata
   {
     float pitch_rate;
     float roll_rate;
     float yaw_rate;
     float lateral_acceleration;
     float longitudinal_acceleration;
     float vertical_acceleration;

     Corrimudata() : pitch_rate(0), roll_rate(0), yaw_rate(0),
     lateral_acceleration(0), longitudinal_acceleration(0), vertical_acceleration(0)
     {
     }
   }
   corrimudata_;
   novatel_gps_msgs::NovatelCorrectedImuData corrimudata_msg_;
   boost::mutex corrimudata_msg_mutex_;

 };
}
#endif
