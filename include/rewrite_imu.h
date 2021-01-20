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

      cov_acc_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      cov_ang_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

    boost::array<double, 9> cov_acc_vec;
    boost::array<double, 9> cov_ang_vec;

    fmat cov_acc;
    fmat cov_ang;

    int counter;

};

#endif
