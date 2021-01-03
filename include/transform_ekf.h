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
#include <tf/tf.h>

using namespace arma;
using namespace std;
using namespace tf2;

class Transform_Ekf {

  public:
    Transform_Ekf(ros::NodeHandle nh) {

      pub  = nh.advertise<nav_msgs::Odometry>("ekf_transf", 100);
      z_angle = 3.04;
      // q_transf = Quaternion(-0.00984011183584, 0.000616954945317, 0.998388208726, 0.055890752745); // 8_1
      q_transf.setRPY(0.0, 0.0, z_angle); // 8_1
      r = {-0.0199131557338, 1.65900934246, 0.900144956133, 1}; // 8_1
      // R_0.getRotation(q_transf);
      // R_0.getRPY(roll, pitch, yaw);
      // tf::Matrix3x3::serializeFloat(R_0);
      // R_z = {{cos(yaw),-sin(yaw),0},{sin(yaw),cos(yaw),0},{0,0,1}};
      // R_y = {{cos(pitch),0,sin(pitch)},{0,1,0},{-sin(pitch),0,cos(pitch)}};
      // R_x = {{1,0,0},{0,cos(z_angle),-sin(z_angle)},{0,-sin(z_angle),cos(z_angle)}};
      // R = inv(R_z*R_y*R_x);
      R = {{cos(z_angle),-sin(z_angle),0},{sin(z_angle),cos(z_angle),0},{0,0,1}};
      // R = {{R_00, R[0][1], R[0][2]}, {R[1][0], R[1][1], R[1][2]}, {R[2][0], R[2][1], R[2][2]}};

    };

    void transf_ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf_old);

  private:

    ros::Publisher pub;
    nav_msgs::Odometry new_ekf;

    float z_angle;
    double roll;
    double pitch;
    double yaw;
    float R_00;

    fvec old_pose;
    fvec old_lin_vel;
    fvec old_ang_vel;
    fvec new_pose;
    fvec new_lin_vel;
    fvec new_ang_vel;
    fvec r;
    fvec old_pose_cov;
    fvec new_pose_cov;
    fvec old_lin_vel_cov;
    fvec new_lin_vel_cov;

    fmat R;
    fmat R_x;
    fmat R_y;
    fmat R_z;
    fmat T;

    Matrix3x3 R_0;
    Quaternion q_transf;
    Quaternion old_quat;
    Quaternion new_quat;

};

#endif
