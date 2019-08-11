#include <simple_fiducial_mapping/localization_node.h>

namespace simple_fiducial_mapping
{
LocalizationNode::LocalizationNode(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle,
                                   const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : node_handle_(node_handle)
  , private_node_handle_(private_node_handle)
  , tf_buffer_(tf_buffer)
  , visualizer_(node_handle)
  , most_recent_pose_(Eigen::Isometry3d::Identity())
{
  // Load map from disk.
  std::string map_filename;
  private_node_handle_.param<std::string>("map_file", map_filename, "");
  assert(loadMapYaml(map_filename, &static_map_));

  // Localization results with a root mean square error larger than this will be thrown out.
  private_node_handle_.param<double>("max_rmse", max_rmse_, std::numeric_limits<double>::infinity());

  // Publish markers for the map fiducials.
  visualizer_.publishGraphVisualization(static_map_);

  ROS_INFO("Published fiducials for map");
}

void LocalizationNode::localizeFromTags(const apriltags2_ros::AprilTagDetectionArrayConstPtr& detections_msg)
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  Eigen::Isometry3d map_H_camera;

  // Lookup transform from robot base link to camera frame. This transform is usually static,
  // so we don't worry much about time.
  geometry_msgs::TransformStamped transform_msg;
  try
  {
    transform_msg = tf_buffer_->lookupTransform(ROBOT_BASE_FRAME, detections_msg->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Unable to lookup transform from %s to %s", ROBOT_BASE_FRAME.c_str(),
             detections_msg->header.frame_id.c_str());
    return;
  }
  Eigen::Isometry3d base_link_H_camera = tf2::transformToEigen(transform_msg);

  std::map<size_t, Eigen::Vector3d> observations = tagDetectionsToObservations(*detections_msg);

  if (localize(observations, most_recent_pose_, max_rmse_, &map_H_camera, &static_map_))
  {
    Eigen::Isometry3d map_H_base_link = map_H_camera * base_link_H_camera.inverse();
    geometry_msgs::TransformStamped transform_stamped_msg = tf2::eigenToTransform(map_H_base_link);
    transform_stamped_msg.header.stamp = detections_msg->header.stamp;
    transform_stamped_msg.header.frame_id = MAP_FRAME;
    transform_stamped_msg.child_frame_id = ROBOT_BASE_FRAME;
    transform_broadcaster.sendTransform(transform_stamped_msg);

    std::map<size_t, Eigen::Isometry3d> camera_pose_map;
    camera_pose_map[0] = map_H_camera;
    visualizer_.publishCameraPose(map_H_camera);
    visualizer_.publishObservationMarkers(static_map_.camera_poses, static_map_.fiducial_positions,
                                          static_map_.observations);

    most_recent_pose_ = map_H_camera;

    ROS_DEBUG_STREAM("Estimated camera pose: " << transform_stamped_msg.transform);
  }
  else
  {
    ROS_ERROR("We cannot localize...");
  }
}

Eigen::Isometry3d LocalizationNode::getMostRecentPose()
{
  return most_recent_pose_;
}
}
