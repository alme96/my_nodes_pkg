#ifndef _EVALUATION_
#define _EVALUATION_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>
#include <iostream>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"

using namespace arma;
using namespace std;
using namespace tf2;

class Evaluation {

  public:
    Evaluation(fmat mat_init, ros::NodeHandle nh) {

      pub  = nh.advertise<geometry_msgs::PoseStamped>("orientations", 100);

      position_ekf = mat_init;
      orientation_ekf = mat_init;
      lin_vel_ekf = mat_init;
      ang_vel_ekf = mat_init;

      position_vicon = mat_init;
      orientation_vicon = mat_init;
      lin_vel_vicon = mat_init;
      ang_vel_vicon = mat_init;

      actual_data = mat_init;
      actual_data_orientation = mat_init;
      error_matrix = mat_init;

      // index_ekf = 0;
      // index_vicon = 0;
      bottom = 0;
      ekf_counter = 0;

    };

    void ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf);
    void viconCallback(const nav_msgs::Odometry::ConstPtr& vicon);
    void error_calculation();

  private:
    fmat position_ekf;
    fmat orientation_ekf;
    fmat lin_vel_ekf;
    fmat ang_vel_ekf;

    fmat position_vicon;
    fmat orientation_vicon;
    fmat lin_vel_vicon;
    fmat ang_vel_vicon;

    fmat actual_data;
    fmat actual_data_orientation;
    fmat error_matrix;

    int ekf_counter;
    int L;
    // int index_ekf;
    // int index_vicon;
    bool bottom;

    Quaternion quat;

    double roll;
    double pitch;
    double yaw;

    ros::Publisher pub;
    geometry_msgs::PoseStamped orientations;

};

#endif
