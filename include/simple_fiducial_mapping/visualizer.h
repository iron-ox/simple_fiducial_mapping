#pragma once

#include <vector>
#include <map>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <simple_fiducial_mapping/mapping.h>

namespace simple_fiducial_mapping
{
static const std::string OBSERVATION_MARKERS_TOPIC = "observation_markers";
static const std::string CAMERA_POSE_TOPIC = "camera_pose";
static const std::string CAMERA_POSES_TOPIC = "camera_poses";
static const std::string CAMERA_POSE_LABELS_TOPIC = "camera_pose_labels";
static const std::string FIDUCIAL_MARKERS_TOPIC = "fiducial_markers";

class Visualizer
{
public:
  Visualizer(ros::NodeHandle node_handle);

  void publishGraphVisualization(const MapGraph& map_graph);

  void publishObservationMarkers(const std::map<size_t, CameraPose>& camera_poses,
                                 const std::map<size_t, FiducialPosition>& fiducial_positions,
                                 const std::map<size_t, Observation>& observations);

  void publishCameraPose(const Eigen::Isometry3d& camera_pose);

  void publishCameraPoses(const std::map<size_t, CameraPose>& camera_poses);
  void publishCameraPoses(const std::map<size_t, Eigen::Isometry3d>& camera_poses);

  void publishFiducialMarkers(const std::map<size_t, FiducialPosition>& fiducial_positions);

private:
  ros::Publisher observation_markers_publisher_;
  ros::Publisher camera_pose_publisher_;
  ros::Publisher camera_poses_publisher_;
  ros::Publisher camera_pose_labels_publisher_;
  ros::Publisher fiducial_markers_publisher_;
};

}  // namespace simple_fiducial_mapping
