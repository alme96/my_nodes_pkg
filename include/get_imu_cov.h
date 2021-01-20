#ifndef _GET_IMU_COV_
#define _GET_IMU_COV_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <armadillo>
#include <iostream>

using namespace arma;
using namespace std;

class Get_Imu_Cov {

  public:
    Get_Imu_Cov(fmat mat_init) {
      counter = 0;
      N = 10001;
      collect_ang_vel = mat_init;
      collect_lin_acc = mat_init;
      data_extractor = mat_init;
    };

    void imucovCallback(const sensor_msgs::Imu::ConstPtr& imu);
    void imu_cov_calculation();

  private:
    int counter;
    int N;
    fmat collect_ang_vel;
    fmat collect_lin_acc;
    fmat data_extractor;
};

#endif
