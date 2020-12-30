#ifndef _REWRITE_IMU_
#define _REWRITE_IMU_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <armadillo>

using namespace arma;

class Rewrite_Imu {

  public:
    Rewrite_Imu(ros::NodeHandle nh) {
      pub  = nh.advertise<sensor_msgs::Imu>("imu_ok", 100);

      lin_acc_offset_x = 0.0;
      lin_acc_offset_y = 0.0;
      lin_acc_offset_z = 0.0;

      ang_vel_offset_x = 0.0;
      ang_vel_offset_y = 0.0;
      ang_vel_offset_z = 0.0;

      counter = 0;

    };

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  private:
    ros::Publisher pub;
    sensor_msgs::Imu new_imu;

    float lin_acc_offset_x;
    float lin_acc_offset_y;
    float lin_acc_offset_z;

    float ang_vel_offset_x;
    float ang_vel_offset_y;
    float ang_vel_offset_z;

    int counter;

};

#endif
