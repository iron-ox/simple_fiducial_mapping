#pragma once

#include <vector>

#include <ros/ros.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <simple_fiducial_mapping/mapping.h>
#include <simple_fiducial_mapping/visualizer.h>

namespace simple_fiducial_mapping
{
static const std::string ROBOT_BASE_FRAME = "base_link";
static const std::string MAP_FRAME = "map";

class LocalizationNode
{
public:
  LocalizationNode(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle,
                   const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);

  void localizeFromTags(const apriltags2_ros::AprilTagDetectionArrayConstPtr& tag_detections_msg);

  Eigen::Isometry3d getMostRecentPose();

private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  Visualizer visualizer_;

  MapGraph static_map_;

  // Store the most recent pose. Used mainly for debugging.
  Eigen::Isometry3d most_recent_pose_;

  double max_rmse_;
};
}
