#include "rewrite_imu.h"

void Rewrite_Imu::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  //Update covariance matrices in order to pass the filter
  new_imu.angular_velocity_covariance = {100, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 100};
  new_imu.linear_acceleration_covariance = {100, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 100};
  //Copy the measurements from the old imu to the new imu message
  new_imu.header = imu->header;
  new_imu.header.stamp = ros::Time::now();
  new_imu.angular_velocity = imu->angular_velocity;
  new_imu.linear_acceleration = imu->linear_acceleration;
  new_imu.orientation = imu->orientation;
  new_imu.orientation_covariance = imu->orientation_covariance;

  pub.publish(new_imu);
}
