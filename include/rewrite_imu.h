#ifndef _REWRITE_IMU_
#define _REWRITE_IMU_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class Rewrite_Imu {

  public:
    Rewrite_Imu(ros::NodeHandle nh) {
      pub  = nh.advertise<sensor_msgs::Imu>("imu_ok", 100);
    };

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  private:
    ros::Publisher pub;
    sensor_msgs::Imu new_imu;

};

#endif
