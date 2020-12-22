#include "rewrite_imu.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "rewrite_imu_node");
  ros::NodeHandle nh;
  Rewrite_Imu rewrite_imu(nh);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, &Rewrite_Imu::imuCallback, &rewrite_imu);
  ros::spin();
}
