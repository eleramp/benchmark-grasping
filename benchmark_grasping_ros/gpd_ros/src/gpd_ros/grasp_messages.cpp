#include <gpd_ros/grasp_messages.h>


benchmark_grasping_ros::BenchmarkGrasp GraspMessages::convertToBenchmarkGraspMsg(const gpd::candidate::Hand& hand, const std_msgs::Header& header)
{
  //first transform grasp orientation wrt camera

  Eigen::Vector3d pos = hand.getPosition();
  Eigen::Vector3d rot_x = hand.getAxis();
  Eigen::Vector3d binormal = hand.getBinormal();
  Eigen::Vector3d rot_z = hand.getApproach();

  Eigen::Vector3d rot_y = rot_z.cross(rot_x);

  Eigen::Matrix4d rotation;
  rotation.setIdentity();

  rotation.col(0).segment(0,3) = rot_x;
  rotation.col(1).segment(0,3) = rot_y;
  rotation.col(2).segment(0,3) = rot_z;
  rotation.col(3).segment(0,3) = pos;

  Eigen::Matrix4d offset;
  offset.setIdentity();
  offset(3,2) = -0.03;

  Eigen::Matrix4d cam_T_grasp = rotation * offset;

  Eigen::Quaterniond quat(Eigen::Matrix3d(cam_T_grasp.block(0,0,3,3)));

  // create message
  benchmark_grasping_ros::BenchmarkGrasp msg;

  msg.pose.header.frame_id = header.frame_id;
  msg.pose.header.stamp = ros::Time::now();


  tf::pointEigenToMsg(hand.getPosition(), msg.pose.pose.position);

  msg.pose.pose.orientation.w = quat.w();
  msg.pose.pose.orientation.x = quat.x();
  msg.pose.pose.orientation.y = quat.y();
  msg.pose.pose.orientation.z = quat.z();

  msg.score.data = hand.getScore();
  msg.width.data = hand.getGraspWidth();

  return msg;

}
