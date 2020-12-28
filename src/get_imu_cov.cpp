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

  // compute mean value
  fmat mean_lin_acc = sum(collect_lin_acc, 0)/(N-1);
  fmat mean_ang_vel = sum(collect_ang_vel, 0)/(N-1);
  // mean_lin_acc.print("Linear acceleration mean value:");
  // mean_ang_vel.print("Angular velocity mean value:");

  // compute covariance
  fmat mean_lin_acc_matrix = collect_lin_acc.zeros();
  fmat mean_ang_vel_matrix = collect_ang_vel.zeros();

  for (int i = 0; i<N-1 ; i++) {
    mean_lin_acc_matrix.row(i) = mean_lin_acc;
    mean_ang_vel_matrix.row(i) = mean_ang_vel;
  }
  mean_lin_acc_matrix.print("Mean linear acceleration matrix: ");

  fmat lin_acc_cov = sum((collect_lin_acc-mean_lin_acc_matrix)%(collect_lin_acc-mean_lin_acc_matrix), 0)/(N-1);
  fmat ang_vel_cov = sum((collect_ang_vel-mean_ang_vel_matrix)%(collect_ang_vel-mean_ang_vel_matrix), 0)/(N-1);

  lin_acc_cov.print("Linear acceleration covariance: ");
  ang_vel_cov.print("Angular velocity covariance: ");

}
