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

    actual_data_orientation(0,0) = ekf->pose.pose.orientation.x;
    actual_data_orientation(0,1) = ekf->pose.pose.orientation.y;
    actual_data_orientation(0,2) = ekf->pose.pose.orientation.z;
    actual_data_orientation(0,3) = ekf->pose.pose.orientation.w;
    // actual_data(0,4) = ros::Time::now().toSec();
    orientation_ekf = join_vert(orientation_ekf, actual_data_orientation);
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

    // cout << "Timing ekf = " << ros::Time::now() << "\n";

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

    actual_data_orientation(0,0) = vicon->pose.pose.orientation.x;
    actual_data_orientation(0,1) = vicon->pose.pose.orientation.y;
    actual_data_orientation(0,2) = vicon->pose.pose.orientation.z;
    actual_data_orientation(0,3) = vicon->pose.pose.orientation.w;
    // actual_data(0,4) = ros::Time::now().toSec();
    orientation_vicon = join_vert(orientation_vicon, actual_data_orientation);


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

  cout << "N = " << N << ", M = " << M << "\n";
  // position_ekf.print("position_ekf final:");
  // position_vicon.print("position_vicon final:");

  if (N<M) {
    cout << "N<M" << "\n";
    //cancel last data
    position_ekf.rows(0,N-1);
    lin_vel_ekf.rows(0,N-1);
    orientation_ekf.rows(0,N-1);
    ang_vel_ekf.rows(0,N-1);

    // for (int i = 1; i<N; i++){
    //   for (int j = i; j<M; j++){
    //     if (position_vicon(i,4) > position_ekf(j-1,4) && position_vicon(i,4) < position_ekf(j,4)) {
    //       // cout << "matching point found" << "\n";
    //       if (abs(position_vicon(i,4)-position_ekf(j-1,4))>abs(position_vicon(i,4)-position_ekf(j,4))) {
    //         // position
    //         error_i = (position_vicon.row(i)-position_ekf.row(j))%(position_vicon.row(i)-position_ekf.row(j));
    //         position_error = join_vert(position_error, error_i.cols(0,2));
    //         // linear velocity
    //         error_i = (lin_vel_vicon.row(i)-lin_vel_ekf.row(j))%(lin_vel_vicon.row(i)-lin_vel_ekf.row(j));
    //         lin_vel_error = join_vert(lin_vel_error, error_i.cols(0,2));
    //         // angular velocity
    //         error_i = (ang_vel_vicon.row(i)-ang_vel_ekf.row(j))%(ang_vel_vicon.row(i)-ang_vel_ekf.row(j));
    //         ang_vel_error = join_vert(ang_vel_error, error_i.cols(0,2));
    //       }
    //       else {
    //         // position
    //         error_i = (position_vicon.row(i)-position_ekf.row(j-1))%(position_vicon.row(i)-position_ekf.row(j-1));
    //         position_error = join_vert(position_error, error_i.cols(0,2));
    //         // linear velocity
    //         error_i = (lin_vel_vicon.row(i)-lin_vel_ekf.row(j-1))%(lin_vel_vicon.row(i)-lin_vel_ekf.row(j-1));
    //         lin_vel_error = join_vert(lin_vel_error, error_i.cols(0,2));
    //         // angular velocity
    //         error_i = (ang_vel_vicon.row(i)-ang_vel_ekf.row(j-1))%(ang_vel_vicon.row(i)-ang_vel_ekf.row(j-1));
    //         ang_vel_error = join_vert(ang_vel_error, error_i.cols(0,2));
    //       }
    //       // error_i.print("error_i:");
    //     }
    //   }
    // }
    if (lin_vel_ekf.n_rows == lin_vel_vicon.n_rows) {

    }
    else {
      ROS_ERROR("dimension error: ekf data has [%i] entries and vicon data has [%i] entries!", lin_vel_ekf.n_rows, lin_vel_vicon.n_rows);
    }

    // Compute mean square error
    distance_error = sum(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)))/N;
    position_error = sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),0)/N);
    lin_vel_error = sqrt(sum((lin_vel_ekf-lin_vel_vicon)%(lin_vel_ekf-lin_vel_vicon),0)/N);
    ang_vel_error = sqrt(sum((ang_vel_ekf-ang_vel_vicon)%(ang_vel_ekf-ang_vel_vicon),0)/N);

    // Compute peak values
    fmat x_error = sort(abs(position_ekf.col(0)-position_vicon.col(0)), "ascend");
    fmat y_error = sort(abs(position_ekf.col(1)-position_vicon.col(1)), "ascend");
    fmat z_error = sort(abs(position_ekf.col(2)-position_vicon.col(2)), "ascend");
    peak_pos_error(0,0) = x_error(0,0);
    peak_pos_error(0,1) = y_error(0,0);
    peak_pos_error(0,2) = z_error(0,0);
    peak_pos_error(1,0) = x_error(N-1,0);
    peak_pos_error(1,1) = y_error(N-1,0);
    peak_pos_error(1,2) = z_error(N-1,0);

    fmat distance_errors = sort(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)), "ascend");
    peak_dist_error(0,0) = distance_errors(0,0);
    peak_dist_error(0,1) = distance_errors(N-1,0);

    x_error = sort(abs(lin_vel_ekf.col(0)-lin_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(lin_vel_ekf.col(1)-lin_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(lin_vel_ekf.col(2)-lin_vel_vicon.col(2)), "ascend");
    peak_linvel_error(0,0) = x_error(0,0);
    peak_linvel_error(0,1) = y_error(0,0);
    peak_linvel_error(0,2) = z_error(0,0);
    peak_linvel_error(1,0) = x_error(N-1,0);
    peak_linvel_error(1,1) = y_error(N-1,0);
    peak_linvel_error(1,2) = z_error(N-1,0);

    x_error = sort(abs(ang_vel_ekf.col(0)-ang_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(ang_vel_ekf.col(1)-ang_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(ang_vel_ekf.col(2)-ang_vel_vicon.col(2)), "ascend");
    peak_anglvel_error(0,0) = x_error(0,0);
    peak_anglvel_error(0,1) = y_error(0,0);
    peak_anglvel_error(0,2) = z_error(0,0);
    peak_anglvel_error(1,0) = x_error(N-1,0);
    peak_anglvel_error(1,1) = y_error(N-1,0);
    peak_anglvel_error(1,2) = z_error(N-1,0);

  }
  else if (N>M) {
    cout << "N>M" << "\n";
    position_vicon.rows(0,M-1);
    lin_vel_vicon.rows(0,M-1);
    orientation_vicon.rows(0,M-1);
    ang_vel_vicon.rows(0,M-1);

    // for (int i = 1; i<M; i++){
    //   for (int j = i; j<N; j++){
    //     if (position_ekf(i,4) >= position_vicon(j-1,4) && position_ekf(i,4) <= position_vicon(j,4)) {
    //       // cout << "matching point found" << "\n";
    //       if (abs(position_ekf(i,4)-position_vicon(j-1,4))>abs(position_ekf(i,4)-position_vicon(j,4))) {
    //         //position
    //         error_i = (position_ekf.row(i)-position_vicon.row(j))%(position_ekf.row(i)-position_vicon.row(j));
    //         position_error = join_vert(position_error, error_i.cols(0,2));
    //         // linear velocity
    //         error_i = (lin_vel_ekf.row(i)-lin_vel_vicon.row(j))%(lin_vel_ekf.row(i)-lin_vel_vicon.row(j));
    //         lin_vel_error = join_vert(lin_vel_error, error_i.cols(0,2));
    //         // angular velocity
    //         error_i = (ang_vel_ekf.row(i)-ang_vel_vicon.row(j))%(ang_vel_ekf.row(i)-ang_vel_vicon.row(j));
    //         ang_vel_error = join_vert(ang_vel_error, error_i.cols(0,2));
    //
    //       }
    //       else {
    //         //position
    //         error_i = (position_ekf.row(i)-position_vicon.row(j-1))%(position_ekf.row(i)-position_vicon.row(j-1));
    //         // linear velocity
    //         error_i = (lin_vel_ekf.row(i)-lin_vel_vicon.row(j-1))%(lin_vel_ekf.row(i)-lin_vel_vicon.row(j-1));
    //         lin_vel_error = join_vert(lin_vel_error, error_i.cols(0,2));
    //         // angular velocity
    //         error_i = (ang_vel_ekf.row(i)-ang_vel_vicon.row(j-1))%(ang_vel_ekf.row(i)-ang_vel_vicon.row(j-1));
    //         ang_vel_error = join_vert(ang_vel_error, error_i.cols(0,2));
    //
    //         position_error = join_vert(position_error, error_i.cols(0,2));
    //       }
    //       // error_i.print("error_i:");
    //     }
    //   }
    // }

    if (lin_vel_ekf.n_rows == lin_vel_vicon.n_rows) {

    }
    else {
      ROS_ERROR("dimension error: ekf data has [%i] entries and vicon data has [%i] entries!", lin_vel_ekf.n_rows, lin_vel_vicon.n_rows);
    }

    // Compute mean square error
    distance_error = sum(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)))/M;
    position_error = sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),0)/M);
    lin_vel_error = sqrt(sum((lin_vel_ekf-lin_vel_vicon)%(lin_vel_ekf-lin_vel_vicon),0)/M);
    ang_vel_error = sqrt(sum((ang_vel_ekf-ang_vel_vicon)%(ang_vel_ekf-ang_vel_vicon),0)/M);

    // Compute peak values
    fmat x_error = sort(abs(position_ekf.col(0)-position_vicon.col(0)), "ascend");
    fmat y_error = sort(abs(position_ekf.col(1)-position_vicon.col(1)), "ascend");
    fmat z_error = sort(abs(position_ekf.col(2)-position_vicon.col(2)), "ascend");
    peak_pos_error(0,0) = x_error(0,0);
    peak_pos_error(0,1) = y_error(0,0);
    peak_pos_error(0,2) = z_error(0,0);
    peak_pos_error(1,0) = x_error(M-1,0);
    peak_pos_error(1,1) = y_error(M-1,0);
    peak_pos_error(1,2) = z_error(M-1,0);

    fmat distance_errors = sort(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)), "ascend");
    peak_dist_error(0,0) = distance_errors(0,0);
    peak_dist_error(0,1) = distance_errors(M-1,0);

    x_error = sort(abs(lin_vel_ekf.col(0)-lin_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(lin_vel_ekf.col(1)-lin_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(lin_vel_ekf.col(2)-lin_vel_vicon.col(2)), "ascend");
    peak_linvel_error(0,0) = x_error(0,0);
    peak_linvel_error(0,1) = y_error(0,0);
    peak_linvel_error(0,2) = z_error(0,0);
    peak_linvel_error(1,0) = x_error(M-1,0);
    peak_linvel_error(1,1) = y_error(M-1,0);
    peak_linvel_error(1,2) = z_error(M-1,0);

    x_error = sort(abs(ang_vel_ekf.col(0)-ang_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(ang_vel_ekf.col(1)-ang_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(ang_vel_ekf.col(2)-ang_vel_vicon.col(2)), "ascend");
    peak_anglvel_error(0,0) = x_error(0,0);
    peak_anglvel_error(0,1) = y_error(0,0);
    peak_anglvel_error(0,2) = z_error(0,0);
    peak_anglvel_error(1,0) = x_error(M-1,0);
    peak_anglvel_error(1,1) = y_error(M-1,0);
    peak_anglvel_error(1,2) = z_error(M-1,0);



  }
  else if (N==M) {
    cout << "N = M" << "\n";
    distance_error = sum(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)))/M;
    position_error = sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),0)/M);
    lin_vel_error = sqrt(sum((lin_vel_ekf-lin_vel_vicon)%(lin_vel_ekf-lin_vel_vicon),0)/M);
    ang_vel_error = sqrt(sum((ang_vel_ekf-ang_vel_vicon)%(ang_vel_ekf-ang_vel_vicon),0)/M);

    // Compute peak values
    fmat x_error = sort(abs(position_ekf.col(0)-position_vicon.col(0)), "ascend");
    fmat y_error = sort(abs(position_ekf.col(1)-position_vicon.col(1)), "ascend");
    fmat z_error = sort(abs(position_ekf.col(2)-position_vicon.col(2)), "ascend");
    peak_pos_error(0,0) = x_error(0,0);
    peak_pos_error(0,1) = y_error(0,0);
    peak_pos_error(0,2) = z_error(0,0);
    peak_pos_error(1,0) = x_error(N-1,0);
    peak_pos_error(1,1) = y_error(N-1,0);
    peak_pos_error(1,2) = z_error(N-1,0);

    fmat distance_errors = sort(sqrt(sum((position_ekf-position_vicon)%(position_ekf-position_vicon),1)), "ascend");
    peak_dist_error(0,0) = distance_errors(0,0);
    peak_dist_error(0,1) = distance_errors(N-1,0);

    x_error = sort(abs(lin_vel_ekf.col(0)-lin_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(lin_vel_ekf.col(1)-lin_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(lin_vel_ekf.col(2)-lin_vel_vicon.col(2)), "ascend");
    peak_linvel_error(0,0) = x_error(0,0);
    peak_linvel_error(0,1) = y_error(0,0);
    peak_linvel_error(0,2) = z_error(0,0);
    peak_linvel_error(1,0) = x_error(N-1,0);
    peak_linvel_error(1,1) = y_error(N-1,0);
    peak_linvel_error(1,2) = z_error(N-1,0);

    x_error = sort(abs(ang_vel_ekf.col(0)-ang_vel_vicon.col(0)), "ascend");
    y_error = sort(abs(ang_vel_ekf.col(1)-ang_vel_vicon.col(1)), "ascend");
    z_error = sort(abs(ang_vel_ekf.col(2)-ang_vel_vicon.col(2)), "ascend");
    peak_anglvel_error(0,0) = x_error(0,0);
    peak_anglvel_error(0,1) = y_error(0,0);
    peak_anglvel_error(0,2) = z_error(0,0);
    peak_anglvel_error(1,0) = x_error(N-1,0);
    peak_anglvel_error(1,1) = y_error(N-1,0);
    peak_anglvel_error(1,2) = z_error(N-1,0);

  }
  // print and save results
  position_error.print("Mean squared position error:");
  distance_error.print("Mean squared distance error:");
  lin_vel_error.print("Mean squared linear velocity error:");
  ang_vel_error.print("Mean squared angular velocity error:");
  peak_pos_error.print("peak_pos_error:");
  peak_dist_error.print("peak_dist_error");
  peak_linvel_error.print("peak_linvel_error:");
  peak_anglvel_error.print("peak_anglvel_error");

  position_error.save("/home/menichea/evaluation/position_error.txt", arma_ascii);
  distance_error.save("/home/menichea/evaluation/distance_error.txt", arma_ascii);
  lin_vel_error.save("/home/menichea/evaluation/lin_vel_error.txt", arma_ascii);
  ang_vel_error.save("/home/menichea/evaluation/ang_vel_error.txt", arma_ascii);
  peak_pos_error.save("/home/menichea/evaluation/peak_pos_error.txt", arma_ascii);
  peak_dist_error.save("/home/menichea/evaluation/peak_dist_error.txt", arma_ascii);
  peak_linvel_error.save("/home/menichea/evaluation/peak_linvel_error.txt", arma_ascii);
  peak_anglvel_error.save("/home/menichea/evaluation/peak_anglvel_error.txt", arma_ascii);
}
