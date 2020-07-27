#include <gpd_ros/grasp_messages.h>

gpd_ros::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  gpd_ros::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::GraspConfig msg;
  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}

benchmark_grasping_ros::BenchmarkGrasp GraspMessages::convertToBenchmarkGraspMsg(const gpd::candidate::Hand& hand, const std_msgs::Header& header)
{
  //first transform grasp orientation wrt camera
  /*
  grasp_pos = parsed_data['position']
  rot_x = parsed_data['axis']
  rot_z = parsed_data['approach']
  rot_y = np.cross(rot_z, rot_x)
  # create T matrix
  cam_T_grasp = np.array([rot_x, rot_y, rot_z, grasp_pos]).transpose()
  cam_T_grasp = np.append(cam_T_grasp, [[0, 0, 0, 1]], axis=0)

  grasp_target_T_panda_ef = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.06], [0, 0, 0, 1]])

  self._cam_T_grasp = np.matmul(cam_T_grasp, grasp_target_T_panda_ef)
  */

  Eigen::Vector3d pos = hand.getPosition();
  Eigen::Vector3d rot_x = hand.getAxis();
  Eigen::Vector3d binormal = hand.getBinormal();
  Eigen::Vector3d rot_z = hand.getApproach();

  Eigen::Vector3d rot_y = rot_z.cross(rot_x);

  Eigen::Matrix4d rotation;
  rotation.setIdentity();

  std::cout << "convertToBenchmarkGraspMsg: \nbinormal " << binormal << std::endl;
  std::cout << "rot_y " << rot_y << std::endl;

  rotation.col(0).segment(0,3) = rot_x;
  rotation.col(1).segment(0,3) = rot_y;
  rotation.col(2).segment(0,3) = rot_z;
  rotation.col(3).segment(0,3) = pos;

  Eigen::Matrix4d offset;
  offset.setIdentity();
  offset(3,2) = -0.06;

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
