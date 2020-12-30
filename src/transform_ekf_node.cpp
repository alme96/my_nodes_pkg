#include "transform_ekf.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "transform_ekf_node");
  ros::NodeHandle nh;
  fvec vec_init(3, fill::zeros);
  Transform_Ekf transform_ekf(nh);
  ros::spin();
}
