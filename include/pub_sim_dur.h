#ifndef _PUB_SIM_DUR_
#define _PUB_SIM_DUR_

#include "ros/ros.h"
#include "std_msgs/Duration.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

class Pub_Sim_Dur {

public:

  Pub_Sim_Dur() {
    pub = nh.advertise<std_msgs::Duration>("/sim_duration", 100);
    sub = nh.subscribe<nav_msgs::Odometry>("/radar/vrpn_client/estimated_odometry", 100, &Pub_Sim_Dur::set_sim_durationCallback, this);
    start_time = ros::Time::now();
    click == 0;
    ros::spin();
  }

  void set_sim_durationCallback(const nav_msgs::Odometry::ConstPtr& vicon) {

    if (click == 0) {
      start_time = vicon->header.stamp;
      std::cout << "Start time set to " << start_time << "\n";
      click++;
    }

    dur.data = vicon->header.stamp - start_time;
    pub.publish(dur);
  }

private:

  ros::Time start_time;
  ros::Publisher pub;
  ros::Subscriber sub;
  std_msgs::Duration dur;
  ros::NodeHandle nh;

  int click;

};

#endif
