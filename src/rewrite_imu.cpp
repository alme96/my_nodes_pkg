#include "rewrite_imu.h"

void Rewrite_Imu::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  if (counter == 0) {
    lin_acc_offset_x = imu->linear_acceleration.x;
    lin_acc_offset_y = imu->linear_acceleration.y;
    lin_acc_offset_z = imu->linear_acceleration.z;

    ang_vel_offset_x = imu->angular_velocity.x;
    ang_vel_offset_y = imu->angular_velocity.y;
    ang_vel_offset_z = imu->angular_velocity.z;

    counter = 1;
  }
  else {
    //Update covariance matrices in order to pass the filter
    new_imu.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    new_imu.linear_acceleration_covariance = {3e-05, 0.0, 0.0, 0.0, 3e-05, 0.0, 0.0, 0.0, 3e-05};
    //Copy the measurements from the old imu to the new imu message
    new_imu.header = imu->header;
    new_imu.header.stamp = ros::Time::now();

    // new_imu.angular_velocity.x = imu->angular_velocity.x - ang_vel_offset_x;
    // new_imu.angular_velocity.y = imu->angular_velocity.y - ang_vel_offset_y;
    // new_imu.angular_velocity.z = imu->angular_velocity.z - ang_vel_offset_z;
    //
    // new_imu.linear_acceleration.x = imu->linear_acceleration.x - lin_acc_offset_x;
    // new_imu.linear_acceleration.y = imu->linear_acceleration.y - lin_acc_offset_y;
    // new_imu.linear_acceleration.z = imu->linear_acceleration.z - lin_acc_offset_z;
    // fmat R = { {-0.9998, 0.0001, 0.0197}, {0.0023, 0.9938, 0.1116}, {-0.0196, 0.1116, -0.9936} }; //  Transformation matrix 8_1
    // vec acc = { imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z };
    // // acc = inv(R)*acc;
    //
    // new_imu.linear_acceleration.x = acc(0,0);
    // new_imu.linear_acceleration.y = acc(1,0);
    // new_imu.linear_acceleration.z = acc(2,0);
    //
    // vec ang = {imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z};
    // // ang = inv(R)*ang;
    // new_imu.angular_velocity.x = ang(0,0);
    // new_imu.angular_velocity.y = ang(1,0);
    // new_imu.angular_velocity.z = ang(2,0);
    //
    //
    new_imu.angular_velocity = imu->angular_velocity;
    new_imu.linear_acceleration = imu->linear_acceleration;
    // new_imu.orientation_covariance = imu->orientation_covariance;

    pub.publish(new_imu);
  }
}
