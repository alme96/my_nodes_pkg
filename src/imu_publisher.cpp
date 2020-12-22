#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<sensor_msgs::Imu>("imu_publisher", 100);
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
