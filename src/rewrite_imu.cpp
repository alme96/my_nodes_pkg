#include "rewrite_imu.h"

void Rewrite_Imu::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  if (counter == 0) {
    cov_acc.load("/home/menichea/catkin_ws/src/my_nodes_pkg/Imu_cov_matrices/cov_acc.txt");
    cov_ang.load("/home/menichea/catkin_ws/src/my_nodes_pkg/Imu_cov_matrices/cov_ang.txt");

    cov_acc.print("IMU linear acceleration covariance matrix:");
    cov_ang.print("IMU angular velocity covariance matrix:");

    int k = 0;
    for (int i = 0; i<2; i++) {
      for (int j = 0; j<2; j++) {
        cov_acc_vec[k] = cov_acc(i,j);
        cov_ang_vec[k] = cov_ang(i,j);
        k++;
      }
      k++;
    }

    counter = 1;
  }
  //Update covariance matrices in order to pass the filter
  new_imu.angular_velocity_covariance = cov_ang_vec;
  new_imu.linear_acceleration_covariance = cov_acc_vec;
  //Copy the measurements from the old imu to the new imu message
  new_imu.header = imu->header;
  new_imu.header.stamp = ros::Time::now();

  new_imu.angular_velocity = imu->angular_velocity;
  new_imu.linear_acceleration = imu->linear_acceleration;
  new_imu.orientation = imu->orientation;
  new_imu.orientation_covariance = imu->orientation_covariance;

  pub.publish(new_imu);
}
