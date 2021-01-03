#include "transform_ekf.h"

void Transform_Ekf::transf_ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf_old)
{

  // extract pose and linear velocity
  old_pose = {ekf_old->pose.pose.position.x, ekf_old->pose.pose.position.y, ekf_old->pose.pose.position.z, 1};
  old_lin_vel = {ekf_old->twist.twist.linear.x, ekf_old->twist.twist.linear.y, ekf_old->twist.twist.linear.z};
  old_ang_vel = {ekf_old->twist.twist.angular.x, ekf_old->twist.twist.angular.y, ekf_old->twist.twist.angular.z};
  convert(ekf_old->pose.pose.orientation, old_quat);



  // create transformation matrix
  fmat filler(1,3, fill::zeros);
  T = join_vert(R,filler);
  T = join_horiz(T,r);

  // transform pose and verlocity
  new_lin_vel = R*old_lin_vel;
  new_ang_vel = R*old_ang_vel;
  new_pose = T*old_pose;
  new_quat = q_transf*old_quat;

  // write new message with transformed values
  new_ekf.pose.pose.position.x = new_pose(0);
  new_ekf.pose.pose.position.y = new_pose(1);
  new_ekf.pose.pose.position.z = new_pose(2);

  new_ekf.twist.twist.linear.x = new_lin_vel(0);
  new_ekf.twist.twist.linear.y = new_lin_vel(1);
  new_ekf.twist.twist.linear.z = new_lin_vel(2);

  new_ekf.twist.twist.angular.x = new_ang_vel(0);
  new_ekf.twist.twist.angular.y = new_ang_vel(1);
  new_ekf.twist.twist.angular.z = new_ang_vel(2);


  // new_ekf.twist = ekf_old->twist;

  convert(new_quat, new_ekf.pose.pose.orientation);

  new_ekf.header.stamp = ros::Time::now();
  new_ekf.header.frame_id = "vicon";

  pub.publish(new_ekf);

}
