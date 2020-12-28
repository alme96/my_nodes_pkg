#include "get_imu_cov.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "get_imu_cov_node");
  ros::NodeHandle nh;
  fmat mat_init(1,3, fill::zeros);
  Get_Imu_Cov get_imu_cov(mat_init);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, &Get_Imu_Cov::imucovCallback, &get_imu_cov);
  ros::spin();
  get_imu_cov.imu_cov_calculation();
}
