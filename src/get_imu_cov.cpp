#include "get_imu_cov.h"

void Get_Imu_Cov::imucovCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  if (counter < N) {
    data_extractor(0,0) = imu->linear_acceleration.x;
    data_extractor(0,1) = imu->linear_acceleration.y;
    data_extractor(0,2) = imu->linear_acceleration.z;
    collect_lin_acc = join_vert(collect_lin_acc, data_extractor);

    data_extractor(0,0) = imu->angular_velocity.x;
    data_extractor(0,1) = imu->angular_velocity.y;
    data_extractor(0,2) = imu->angular_velocity.z;
    collect_ang_vel = join_vert(collect_ang_vel, data_extractor);
    counter++;
  }
  else if (counter == N) {
    cout << counter << " data points were collected from imu and are ready to be evaluated." << "\n";
    counter++;
  }
  else {

  }
}

void Get_Imu_Cov::imu_cov_calculation() {

  // delete first row
  collect_ang_vel = collect_ang_vel.rows(1,N-1);
  collect_lin_acc = collect_lin_acc.rows(1,N-1);
  
  // compute covariance matrix
  fmat cov_acc = cov(collect_lin_acc);
  fmat cov_ang = cov(collect_ang_vel);

  // compute mean value
  //fmat mean_lin_acc = sum(collect_lin_acc, 0)/(N-1);
  //fmat mean_ang_vel = sum(collect_ang_vel, 0)/(N-1);
  //mean_lin_acc.print("Linear acceleration mean value:");
  //mean_ang_vel.print("Angular velocity mean value:");

  // compute covariance
  //fmat mean_lin_acc_matrix(collect_lin_acc.n_rows, collect_lin_acc.n_cols, fill::zeros);
  //fmat mean_ang_vel_matrix(collect_ang_vel.n_rows, collect_ang_vel.n_cols, fill::zeros);

  //for (int i = 0; i<N-1 ; i++) {
    //mean_lin_acc_matrix.row(i) = mean_lin_acc;
    //mean_ang_vel_matrix.row(i) = mean_ang_vel;
  //}

  // collect_lin_acc.print("collected linear accelerations: ");
  // collect_ang_vel.print("collected angular velocities: ");
  //
  // mean_lin_acc_matrix.print("Mean linear acceleration matrix: ");
  // mean_ang_vel_matrix.print("Mean angular velocity matrix: ");

  // fmat diff = collect_lin_acc + mean_lin_acc_matrix;
  // diff.print("diff:");
  // fmat diff_diff = diff%diff;
  // diff_diff.print("diff^2");

  //fmat lin_acc_cov = sum((collect_lin_acc-mean_lin_acc_matrix)%(collect_lin_acc-mean_lin_acc_matrix), 0)/(N-1);
  //fmat ang_vel_cov = sum((collect_ang_vel-mean_ang_vel_matrix)%(collect_ang_vel-mean_ang_vel_matrix), 0)/(N-1);

  //lin_acc_cov.print("Linear acceleration variance: ");
  //ang_vel_cov.print("Angular velocity variance: ");
  cov_acc.print("Linear acceleration covariance matrix:");
  cov_ang.print("Angular velocity covariance matrix:");
  
  cov_acc.save("/home/menichea/catkin_ws/src/my_nodes_pkg/Imu_cov_matrices/cov_acc.txt", arma_ascii);
  cov_ang.save("/home/menichea/catkin_ws/src/my_nodes_pkg/Imu_cov_matrices/cov_ang.txt", arma_ascii);
}
