#include "transform_ekf.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "transform_ekf_node");
  ros::NodeHandle nh;
  Transform_Ekf transform_ekf(nh);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 100, &Transform_Ekf::transf_ekfCallback, &transform_ekf);
  ros::spin();
}
