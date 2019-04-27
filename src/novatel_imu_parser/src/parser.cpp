#include <boost/assign.hpp>
#include "novatel_imu_parser/parser.h"
#include <iostream>

namespace parser
{
  Novatel_imu::Novatel_imu()
  {
    corrimudata_sub_ = nh_.subscribe("/corrimudata", 1, &Novatel_imu::corrimudataCallback, this);
    inspva_sub_ = nh_.subscribe("/inspva", 1, &Novatel_imu::inspvaCallback, this);
    insstdev_sub_ = nh_.subscribe("/insstdev", 1, &Novatel_imu::insstdevCallback, this);
    compiled_imu_pub = nh_.advertise<sensor_msgs::Imu>("/compiled_imu", 1000);
  }

  void Novatel_imu::subscribeToIMU()
  {
    boost::mutex::scoped_lock corrimudata_msg_lock(corrimudata_msg_mutex_, boost::try_to_lock);
    sensor_msgs::Imu imu_msg;
    int imu_rate_ = 20;
    float deg2rad = M_PI / 180;
    corrimudata_.pitch_rate = corrimudata_msg_.pitch_rate * imu_rate_;
    corrimudata_.roll_rate = corrimudata_msg_.roll_rate * imu_rate_;
    corrimudata_.yaw_rate = corrimudata_msg_.yaw_rate * imu_rate_;
    corrimudata_.lateral_acceleration = corrimudata_msg_.lateral_acceleration;
    corrimudata_.longitudinal_acceleration = corrimudata_msg_.longitudinal_acceleration;
    corrimudata_.vertical_acceleration = corrimudata_msg_.vertical_acceleration;
    boost::mutex::scoped_lock inspva_data_msg_lock(inspva_data_msg_mutex_, boost::try_to_lock);
    inspva_data_.pitch = inspva_data_msg_.pitch;
    inspva_data_.roll = inspva_data_msg_.roll;
    inspva_data_.azimuth = inspva_data_msg_.azimuth;
    inspva_data_.x_vel = inspva_data_msg_.north_velocity;
    inspva_data_.y_vel = inspva_data_msg_.east_velocity;
    inspva_data_.z_vel = inspva_data_msg_.up_velocity;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(inspva_data_.roll * deg2rad,
                                                  -(inspva_data_.pitch) * deg2rad,
                                                  -(inspva_data_.azimuth) * deg2rad);
    imu_msg.orientation_covariance[0] = pow(insstdev_data_.pitch_dev, 2);
    imu_msg.orientation_covariance[4] =pow(insstdev_data_.roll_dev, 2);
    imu_msg.orientation_covariance[8] = pow(insstdev_data_.azimuth_dev, 2);
    imu_msg.angular_velocity.x = corrimudata_.roll_rate;
    imu_msg.angular_velocity.y = corrimudata_.pitch_rate;
    imu_msg.angular_velocity.z = corrimudata_.yaw_rate;
    imu_msg.angular_velocity_covariance[0] =
    imu_msg.angular_velocity_covariance[4] =
    imu_msg.angular_velocity_covariance[8] = 1e-3;
    imu_msg.linear_acceleration.x = corrimudata_.lateral_acceleration;
    imu_msg.linear_acceleration.y = corrimudata_.longitudinal_acceleration;
    imu_msg.linear_acceleration.z = corrimudata_.vertical_acceleration;
    imu_msg.linear_acceleration_covariance[0] =
    imu_msg.linear_acceleration_covariance[4] =
    imu_msg.linear_acceleration_covariance[8] = 1e-3;
    compiled_imu_pub.publish(imu_msg);
  }


  void Novatel_imu::corrimudataCallback(const novatel_gps_msgs::NovatelCorrectedImuData msg)
  {
    // Update the feedback message pointer to point to the current message. Block
    // until the control thread is not using the lock.
    boost::mutex::scoped_lock corrimu_lock(corrimudata_msg_mutex_);
    corrimudata_msg_ = msg;
  }

  void Novatel_imu::inspvaCallback(const novatel_gps_msgs::Inspva msg)
  {
    // Update the feedback message pointer to point to the current message. Block
    // until the control thread is not using the lock.
    boost::mutex::scoped_lock inspva_lock(inspva_data_msg_mutex_);
    inspva_data_msg_ = msg;
  }

  void Novatel_imu::insstdevCallback(const novatel_gps_msgs::Insstdev msg)
  {
    // Update the feedback message pointer to point to the current message. Block
    // until the control thread is not using the lock.
    boost::mutex::scoped_lock inscov_lock(insstdev_data_msg_mutex_);
    insstdev_data_msg_ = msg;
  }
}
