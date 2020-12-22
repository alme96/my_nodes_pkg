#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("I heard imu");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("imu", 100, imuCallback);
  ros::spin();

  return 0;
}
