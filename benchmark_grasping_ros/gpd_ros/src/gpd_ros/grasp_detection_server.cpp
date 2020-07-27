#include <gpd_ros/grasp_detection_server.h>


GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node, std::string config_file,
                                           bool publish_rviz, std::string grasp_publisher_name)
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  view_point_ << 0.0, 0.0, 0.0;

  ROS_INFO_STREAM("GPD GraspDetectionServer: config file is " << config_file);

  grasp_detector_ = new gpd::GraspDetector(config_file);

  if (publish_rviz == true)
  {
    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  if (publish_rviz == true)
  {
    std::string rviz_topic = "rviz_gpd_grasps";
    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  // Advertise ROS topic for detected grasps.
  if (!grasp_publisher_name.empty())
  {
    grasps_pub_ = node.advertise<geometry_msgs::PoseStamped>(grasp_publisher_name, 10);
  }

  node.getParam("workspace", workspace_); // this wariable is not used I think

}

bool GraspDetectionServer::detectBenchGrasps(benchmark_grasping_ros::GraspPlannerCloud::Request& req,
                                        benchmark_grasping_ros::GraspPlannerCloud::Response& res)
{
  ROS_INFO("Received service request from benchmark...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;

  const sensor_msgs::PointCloud2 & cloud_ros = req.cloud;

  // Set view points.
  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;
  view_points.col(0) << req.view_point.position.x, req.view_point.position.y, req.view_point.position.z;

  // Set point cloud.
  if (cloud_ros.fields.size() == 6 && cloud_ros.fields[3].name == "normal_x"
    && cloud_ros.fields[4].name == "normal_y" && cloud_ros.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_ros, *cloud);

    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(req.cloud, *cloud);

    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
  }

  frame_ = cloud_ros.header.frame_id;

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }



    // Create benchmark grasp reply.
    benchmark_grasping_ros::BenchmarkGrasp bench_grasp = GraspMessages::convertToBenchmarkGraspMsg(*grasps[0], cloud_camera_header_);

    // Publish grasp on topic
    grasps_pub_.publish(bench_grasp.pose);

    res.grasp = bench_grasp;
    ROS_INFO_STREAM("Detected grasp.");
    return true;
  }

  ROS_WARN("No grasps detected!");
  return false;
}


bool GraspDetectionServer::detectGrasps(gpd_ros::detect_grasps::Request& req, gpd_ros::detect_grasps::Response& res)
{
  ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;
  const gpd_ros::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources;

  // Set view points.
  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
  for (int i = 0; i < cloud_sources.view_points.size(); i++)
  {
    view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y,
      cloud_sources.view_points[i].z;
  }

  // Set point cloud.
  if (cloud_sources.cloud.fields.size() == 6 && cloud_sources.cloud.fields[3].name == "normal_x"
    && cloud_sources.cloud.fields[4].name == "normal_y" && cloud_sources.cloud.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
    std::cout << "view_points:\n" << view_points << "\n";
  }

  // Set the indices at which to sample grasp candidates.
  std::vector<int> indices(req.cloud_indexed.indices.size());
  for (int i=0; i < indices.size(); i++)
  {
    indices[i] = req.cloud_indexed.indices[i].data;
  }
  cloud_camera_->setSampleIndices(indices);

  frame_ = req.cloud_indexed.cloud_sources.cloud.header.frame_id;

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << req.cloud_indexed.indices.size() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res.grasp_configs = selected_grasps_msg;
    ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
    return true;
  }

  ROS_WARN("No grasps detected!");
  return false;
}

int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "detect_grasps_server");
  ros::NodeHandle node("~");

  std::string grasp_service_name;
  std::string grasp_publisher_name;
  std::string config_file;
  node.param<std::string>("grasp_planner_service_name", grasp_service_name, "gpd_grasp_planner_service");
  node.param<std::string>("grasp_publisher_name", grasp_publisher_name, "");

  node.getParam("config_file", config_file);

  ROS_INFO_STREAM("Grasp detection server:\n grasp_planner_service_name " << grasp_service_name <<
            "\ngrasp_publisher_name "<< grasp_publisher_name <<
            "\nconfig_file " << config_file);

  bool publish_rviz;
  node.param<bool>("publish_rviz", publish_rviz, false);

  GraspDetectionServer grasp_detection_server(node, config_file);

  ros::ServiceServer service =
      node.advertiseService(grasp_service_name, &GraspDetectionServer::detectBenchGrasps,
                                                     &grasp_detection_server);

  ROS_INFO("GPD Grasp detection service is waiting for a point cloud ...");

  ros::spin();

  return 0;
}
