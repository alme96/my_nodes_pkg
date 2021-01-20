#include "evaluation.h"

void Evaluation::ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf)
{
  if (ekf_counter == 0) {
    //Save pose and velocity data from ekf estimation and add it to the ekf data collector matrix
    actual_data(0,0) = ekf->pose.pose.position.x;
    actual_data(0,1) = ekf->pose.pose.position.y;
    actual_data(0,2) = ekf->pose.pose.position.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    // ROS_INFO("saved ekf time = [%f]", actual_data(0,4));
    // cout << "Time now = " << ros::Time::now() << "\n";
    // cout << "Time in sec float = " << ros::Time::now().toSec();
    position_ekf = join_vert(position_ekf, actual_data);
    // position_ekf.print("position_ekf:");

    // actual_data_orientation(0,0) = ekf->pose.pose.orientation.x;
    // actual_data_orientation(0,1) = ekf->pose.pose.orientation.y;
    // actual_data_orientation(0,2) = ekf->pose.pose.orientation.z;
    // actual_data_orientation(0,3) = ekf->pose.pose.orientation.w;

    convert(ekf->pose.pose.orientation, quat);
    // quat.setRPY(roll, pitch, yaw);
    Matrix3x3(quat).getRPY(roll, pitch, yaw);

    actual_data(0,0) = roll;
    actual_data(0,1) = pitch;
    actual_data(0,2) = yaw;

    orientations.pose.position.x = roll;
    orientations.pose.position.y = pitch;
    orientations.pose.position.z = yaw;

    orientation_ekf = join_vert(orientation_ekf, actual_data);
    // orientation_ekf.print("orientation_ekf:");

    actual_data(0,0) = ekf->twist.twist.linear.x;
    actual_data(0,1) = ekf->twist.twist.linear.y;
    actual_data(0,2) = ekf->twist.twist.linear.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    lin_vel_ekf = join_vert(lin_vel_ekf, actual_data);

    actual_data(0,0) = ekf->twist.twist.angular.x;
    actual_data(0,1) = ekf->twist.twist.angular.y;
    actual_data(0,2) = ekf->twist.twist.angular.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    ang_vel_ekf = join_vert(ang_vel_ekf, actual_data);

    bottom = 1;



    ekf_counter++;
  }
}

void Evaluation::viconCallback(const nav_msgs::Odometry::ConstPtr& vicon)
{
  if (bottom) {
    //Save pose and velocity data from vicon estimation and add it to the vicon data collector matrix
    actual_data(0,0) = vicon->pose.pose.position.x;
    actual_data(0,1) = vicon->pose.pose.position.y;
    actual_data(0,2) = vicon->pose.pose.position.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    // ROS_INFO("saved vicon time = [%f]", actual_data(0,4));
    // cout << "Time now = " << ros::Time::now() << "\n";
    // cout << "Time in sec float = " << ros::Time::now().toSec() << "\n";

    position_vicon = join_vert(position_vicon, actual_data);

    convert(vicon->pose.pose.orientation, quat);
    // quat.setRPY(roll, pitch, yaw);
    Matrix3x3(quat).getRPY(roll, pitch, yaw);

    actual_data(0,0) = roll;
    actual_data(0,1) = pitch;
    actual_data(0,2) = yaw;

    orientations.pose.orientation.x = roll;
    orientations.pose.orientation.y = pitch;
    orientations.pose.orientation.z = yaw;

    orientations.header.stamp = ros::Time::now();

    pub.publish(orientations);

    orientation_vicon = join_vert(orientation_vicon, actual_data);


    actual_data(0,0) = vicon->twist.twist.linear.x;
    actual_data(0,1) = vicon->twist.twist.linear.y;
    actual_data(0,2) = vicon->twist.twist.linear.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    lin_vel_vicon = join_vert(lin_vel_vicon, actual_data);

    actual_data(0,0) = vicon->twist.twist.angular.x;
    actual_data(0,1) = vicon->twist.twist.angular.y;
    actual_data(0,2) = vicon->twist.twist.angular.z;
    // actual_data(0,3) = 0;
    // actual_data(0,4) = ros::Time::now().toSec();
    ang_vel_vicon = join_vert(ang_vel_vicon, actual_data);

    bottom = 0;

    // cout << "Timing vicon = " << ros::Time::now() << "\n";

    ekf_counter = 0;
  }
}

void Evaluation::error_calculation()
{
  // This function computes the mean square error of position, orientation, linear and angular velocity.
  int N = lin_vel_vicon.n_rows; // number of saved vicon data
  int M = lin_vel_ekf.n_rows; // number of saved ekf data

  fmat peak_pos_error(2,3, fill::zeros);
  fmat peak_dist_error(1,2, fill::zeros);
  fmat peak_orient_error(2,3, fill::zeros);
  fmat peak_linvel_error(2,3, fill::zeros);
  fmat peak_anglvel_error(2,3, fill::zeros);

  fmat position_error(1,3, fill::zeros);
  fmat distance_error(1,3, fill::zeros);
  fmat lin_vel_error(1,3, fill::zeros);
  fmat ang_vel_error(1,3, fill::zeros);
  fmat orientation_error;

  fmat final_position_error;
  fmat final_lin_vel_error;
  fmat final_ang_vel_error;
  fmat final_orientation_error;
  fmat final_distance_error;

  cout << "N = " << N << ", M = " << M << "\n";
  // position_ekf.print("position_ekf final:");
  // position_vicon.print("position_vicon final:");

  if (N<M) {
    cout << "N<M" << "\n";
    //cancel last data
    position_ekf = position_ekf.rows(0,N-1);
    lin_vel_ekf = lin_vel_ekf.rows(0,N-1);
    orientation_ekf = orientation_ekf.rows(0,N-1);
    ang_vel_ekf =  ang_vel_ekf.rows(0,N-1);

    L = N;
  }
  else if (N>M) {
    cout << "N>M" << "\n";
    position_vicon = position_vicon.rows(0,M-1);
    lin_vel_vicon = lin_vel_vicon.rows(0,M-1);
    orientation_vicon = orientation_vicon.rows(0,M-1);
    ang_vel_vicon = ang_vel_vicon.rows(0,M-1);

    L = M;
  }
  else if (N==M) {
    cout << "N = M" << "\n";

    L = N;
  }

  if (lin_vel_ekf.n_rows == lin_vel_vicon.n_rows) {

  }
  else {
    ROS_ERROR("dimension error: ekf data has [%i] entries and vicon data has [%i] entries!", lin_vel_ekf.n_rows, lin_vel_vicon.n_rows);
  }


  // Compute mean square error
  distance_error = sum(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)))/L;
  position_error = sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),0)/L);
  lin_vel_error = sqrt(sum((lin_vel_ekf-lin_vel_vicon)%(lin_vel_ekf-lin_vel_vicon),0)/L);
  ang_vel_error = sqrt(sum((ang_vel_ekf-ang_vel_vicon)%(ang_vel_ekf-ang_vel_vicon),0)/L);
  orientation_error = sqrt(sum((orientation_ekf-orientation_vicon)%(orientation_ekf-orientation_vicon),0)/L);

  // Compute peak values
  fmat x_error = sort(abs(position_ekf.col(0)-position_vicon.col(0)), "ascend");
  fmat y_error = sort(abs(position_ekf.col(1)-position_vicon.col(1)), "ascend");
  fmat z_error = sort(abs(position_ekf.col(2)-position_vicon.col(2)), "ascend");
  peak_pos_error(0,0) = x_error(0,0);
  peak_pos_error(0,1) = y_error(0,0);
  peak_pos_error(0,2) = z_error(0,0);
  peak_pos_error(1,0) = x_error(L-1,0);
  peak_pos_error(1,1) = y_error(L-1,0);
  peak_pos_error(1,2) = z_error(L-1,0);

  fmat roll_error = sort(abs(orientation_ekf.col(0)-orientation_vicon.col(0)), "ascend");
  fmat pitch_error = sort(abs(orientation_ekf.col(1)-orientation_vicon.col(1)), "ascend");
  fmat yaw_error = sort(abs(orientation_ekf.col(2)-orientation_vicon.col(2)), "ascend");
  peak_orient_error(0,0) = roll_error(0,0);
  peak_orient_error(0,1) = pitch_error(0,0);
  peak_orient_error(0,2) = yaw_error(0,0);
  peak_orient_error(1,0) = roll_error(L-1,0);
  peak_orient_error(1,1) = pitch_error(L-1,0);
  peak_orient_error(1,2) = yaw_error(L-1,0);

  fmat distance_errors = sort(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)), "ascend");
  peak_dist_error(0,0) = distance_errors(0,0);
  peak_dist_error(0,1) = distance_errors(L-1,0);

  x_error = sort(abs(lin_vel_ekf.col(0)-lin_vel_vicon.col(0)), "ascend");
  y_error = sort(abs(lin_vel_ekf.col(1)-lin_vel_vicon.col(1)), "ascend");
  z_error = sort(abs(lin_vel_ekf.col(2)-lin_vel_vicon.col(2)), "ascend");
  peak_linvel_error(0,0) = x_error(0,0);
  peak_linvel_error(0,1) = y_error(0,0);
  peak_linvel_error(0,2) = z_error(0,0);
  peak_linvel_error(1,0) = x_error(L-1,0);
  peak_linvel_error(1,1) = y_error(L-1,0);
  peak_linvel_error(1,2) = z_error(L-1,0);

  x_error = sort(abs(ang_vel_ekf.col(0)-ang_vel_vicon.col(0)), "ascend");
  y_error = sort(abs(ang_vel_ekf.col(1)-ang_vel_vicon.col(1)), "ascend");
  z_error = sort(abs(ang_vel_ekf.col(2)-ang_vel_vicon.col(2)), "ascend");
  peak_anglvel_error(0,0) = x_error(0,0);
  peak_anglvel_error(0,1) = y_error(0,0);
  peak_anglvel_error(0,2) = z_error(0,0);
  peak_anglvel_error(1,0) = x_error(L-1,0);
  peak_anglvel_error(1,1) = y_error(L-1,0);
  peak_anglvel_error(1,2) = z_error(L-1,0);

  //Final errors
  final_position_error = abs(position_ekf.row(L-1)-position_vicon.row(L-1));
  final_lin_vel_error = abs(lin_vel_ekf.row(L-1)-lin_vel_vicon.row(L-1));
  final_ang_vel_error = abs(ang_vel_ekf.row(L-1)-ang_vel_vicon.row(L-1));
  final_orientation_error = abs(orientation_ekf.row(L-1)-orientation_vicon(L-1));
  final_distance_error = sqrt(sum(final_position_error%final_position_error,1));

  // Quaternions for pose error

  // print and save results
  position_error.print("Mean squared position error:");
  distance_error.print("Mean squared distance error:");
  lin_vel_error.print("Mean squared linear velocity error:");
  ang_vel_error.print("Mean squared angular velocity error:");
  orientation_error.print("Mean squared orientation error:");

  peak_pos_error.print("peak_pos_error:");
  peak_dist_error.print("peak_dist_error");
  peak_linvel_error.print("peak_linvel_error:");
  peak_anglvel_error.print("peak_anglvel_error");
  peak_orient_error.print("peak_orient_error:");

  final_lin_vel_error.print("final_lin_vel_error:");
  final_ang_vel_error.print("final_ang_vel_error:");
  final_distance_error.print("final_distance_error:");
  final_position_error.print("final_position_error:");
  final_orientation_error.print("final_orientation_error:");

  position_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/position_error.txt", arma_ascii);
  distance_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/distance_error.txt", arma_ascii);
  lin_vel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/lin_vel_error.txt", arma_ascii);
  ang_vel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/ang_vel_error.txt", arma_ascii);
  orientation_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/orientation_error.txt", arma_ascii);

  peak_pos_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/peak_pos_error.txt", arma_ascii);
  peak_dist_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/peak_dist_error.txt", arma_ascii);
  peak_linvel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/peak_linvel_error.txt", arma_ascii);
  peak_anglvel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/peak_anglvel_error.txt", arma_ascii);
  peak_orient_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/peak_orient_error.txt", arma_ascii);

  final_lin_vel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/final_lin_vel_error.txt", arma_ascii);
  final_ang_vel_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/final_ang_vel_error.txt", arma_ascii);
  final_distance_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/final_distance_error.txt", arma_ascii);
  final_position_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/final_position_error.txt", arma_ascii);
  final_orientation_error.save("/home/menichea/Desktop/SemesterProject_RadialInertialStateEstimation/results/only_radar/final_orientation_error.txt", arma_ascii);

}
