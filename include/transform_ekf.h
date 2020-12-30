#ifndef _TRANSFORM_EKF_
#define _TRANSFORM_EKF_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

using namespace arma;
using namespace std;
using namespace tf2;

class Transform_Ekf {

  public:
    Transform_Ekf(ros::NodeHandle nh) : tf_(), target_frame_(vicon) {

      pub  = nh.advertise<nav_msgs::Odometry>("ekf_transf", 100);
      sub.subscribe(n_, "/odometry/filtered", 100);
      tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(sub, tf_, target_frame_, 100);

      // z_angl = 3.04;

      // q_transf.setRPY(0.0,0.0,z_angl);
      // r = {-0.0199131557338, 1.65900934246, 0.900144956133, 1};
      // R = {{cos(z_angl), -sin(z_angl), 0}, {sin(z_angl), cos(z_angl), 0}, {0, 0, 1}};

    };

    void transf_ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf_old);

  private:

    ros::Publisher pub;
    nav_msgs::Odometry new_ekf;
    message_filters::Subscriber<nav_msgs::Odometry> sub;
    tf::TransformListener tf_;
    tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
    std::string target_frame_;
    ros::NodeHandle n_;


    // float z_angl;
    //
    // fvec old_pose;
    // fvec old_lin_vel;
    // fvec old_ang_vel;
    // fvec new_pose;
    // fvec new_lin_vel;
    // fvec new_ang_vel;
    // fvec r;
    // fvec old_pose_cov;
    // fvec new_pose_cov;
    // fvec old_lin_vel_cov;
    // fvec new_lin_vel_cov;
    //
    // fmat R;
    // fmat T;
    //
    // Quaternion q_transf;
    // Quaternion old_quat;
    // Quaternion new_quat;

};

#endif
