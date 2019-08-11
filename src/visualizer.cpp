#include <simple_fiducial_mapping/visualizer.h>

namespace simple_fiducial_mapping
{
Visualizer::Visualizer(ros::NodeHandle node_handle)
{
  std::string camera_viz_topic;

  observation_markers_publisher_ = node_handle.advertise<visualization_msgs::Marker>(
      OBSERVATION_MARKERS_TOPIC, 1 /* queue_size */, true /* latched */);
  camera_pose_publisher_ =
      node_handle.advertise<geometry_msgs::Pose>(CAMERA_POSE_TOPIC, 1 /* queue_size */, true /* latched */);
  camera_poses_publisher_ =
      node_handle.advertise<geometry_msgs::PoseArray>(CAMERA_POSES_TOPIC, 1 /* queue_size */, true /* latched */);
  camera_pose_labels_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      CAMERA_POSE_LABELS_TOPIC, 1 /* queue_size */, true /* latched */);
  fiducial_markers_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      FIDUCIAL_MARKERS_TOPIC, 1 /* queue_size */, true /* latched */);
}

void Visualizer::publishGraphVisualization(const MapGraph& map_graph)
{
  publishObservationMarkers(map_graph.camera_poses, map_graph.fiducial_positions, map_graph.observations);

  publishCameraPoses(map_graph.camera_poses);

  publishFiducialMarkers(map_graph.fiducial_positions);
}

void Visualizer::publishObservationMarkers(const std::map<size_t, CameraPose>& camera_poses,
                                           const std::map<size_t, FiducialPosition>& fiducial_positions,
                                           const std::map<size_t, Observation>& observations)
{
  // publish a line marker from camera position to the positions of all
  // tags it sees
  visualization_msgs::Marker observation_markers_msg;
  observation_markers_msg.header.frame_id = "map";
  observation_markers_msg.type = visualization_msgs::Marker::LINE_LIST;
  observation_markers_msg.id = 1;
  observation_markers_msg.color.r = 0.0;
  observation_markers_msg.color.g = 0.0;
  observation_markers_msg.color.b = 0.7;
  observation_markers_msg.color.a = 0.7;
  observation_markers_msg.scale.x = 0.05;
  for (const auto& observation_pair : observations)
  {
    size_t camera_pose_id = observation_pair.second.camera_pose_id;
    size_t fiducial_id = observation_pair.second.fiducial_id;
    geometry_msgs::Point camera_position_msg;
    const Eigen::Vector3d camera_position = camera_poses.at(camera_pose_id).position;
    camera_position_msg = tf2::toMsg(camera_position);

    // We can only draw the observation if this tag is in the map.
    const auto fiducial_position_iter = fiducial_positions.find(fiducial_id);
    if (fiducial_position_iter != fiducial_positions.end())
    {
      geometry_msgs::Point fiducial_position;
      Eigen::Vector3d fiducial_position_eigen = fiducial_position_iter->second.position;
      fiducial_position.x = fiducial_position_eigen.x();
      fiducial_position.y = fiducial_position_eigen.y();
      fiducial_position.z = fiducial_position_eigen.z();
      observation_markers_msg.points.push_back(camera_position_msg);
      observation_markers_msg.points.push_back(fiducial_position);
    }
  }
  observation_markers_publisher_.publish(observation_markers_msg);
}

void Visualizer::publishCameraPose(const Eigen::Isometry3d& camera_pose)
{
  camera_pose_publisher_.publish(tf2::toMsg(camera_pose));
}

void Visualizer::publishCameraPoses(const std::map<size_t, CameraPose>& camera_poses)
{
  std::map<size_t, Eigen::Isometry3d> eigen_camera_poses;
  for (const auto& camera_pose_pair : camera_poses)
  {
    eigen_camera_poses[camera_pose_pair.first] = computeCameraTransform(camera_pose_pair.second);
  }
  publishCameraPoses(eigen_camera_poses);
}

void Visualizer::publishCameraPoses(const std::map<size_t, Eigen::Isometry3d>& camera_poses)
{
  // publish camera poses for visualization
  geometry_msgs::PoseArray camera_poses_msg;
  visualization_msgs::MarkerArray camera_pose_labels_msg;
  camera_poses_msg.header.frame_id = "map";
  for (const auto& camera_pose_pair : camera_poses)
  {
    geometry_msgs::Pose pose = tf2::toMsg(camera_pose_pair.second);
    visualization_msgs::Marker camera_pose_label_msg;
    camera_pose_label_msg.header.stamp = ros::Time::now();
    camera_pose_label_msg.header.frame_id = "map";
    camera_pose_label_msg.id = camera_pose_pair.first;
    camera_pose_label_msg.ns = "camera_pose_labels";
    camera_pose_label_msg.action = visualization_msgs::Marker::ADD;
    camera_pose_label_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    camera_pose_label_msg.pose = pose;
    camera_pose_label_msg.pose.position.z += 0.2;  // Position label above pose marker.
    camera_pose_label_msg.scale.z = 0.03;          // Text font height in meters.
    camera_pose_label_msg.color.r = 1.0;
    camera_pose_label_msg.color.g = 1.0;
    camera_pose_label_msg.color.b = 1.0;
    camera_pose_label_msg.color.a = 1.0;
    char pose_id_text[32];  // Can hold IDs up to 10^32 - should be plenty.
    snprintf(pose_id_text, 32, "%zu", camera_pose_pair.first);
    camera_pose_label_msg.text = pose_id_text;
    camera_pose_labels_msg.markers.push_back(camera_pose_label_msg);

    camera_poses_msg.poses.push_back(pose);
  }
  camera_poses_publisher_.publish(camera_poses_msg);
  camera_pose_labels_publisher_.publish(camera_pose_labels_msg);
}

void Visualizer::publishFiducialMarkers(const std::map<size_t, FiducialPosition>& fiducial_positions)
{
  // Cubes to show fiducial positions in the map.
  visualization_msgs::MarkerArray fiducial_markers;
  for (const auto& tag_pair : fiducial_positions)
  {
    Eigen::Vector3d fiducial_position = tag_pair.second.position;
    visualization_msgs::Marker marker;
    marker.id = tag_pair.first;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = fiducial_position.x();
    marker.pose.position.y = fiducial_position.y();
    marker.pose.position.z = fiducial_position.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.header.frame_id = "map";
    fiducial_markers.markers.push_back(marker);
  }

  // Text of the fiducial ID above each fiducial
  for (const auto& tag_pair : fiducial_positions)
  {
    Eigen::Vector3d fiducial_position = tag_pair.second.position;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = tag_pair.first;
    marker.ns = "fiducial_ids";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.pose.position.x = fiducial_position.x();
    marker.pose.position.y = fiducial_position.y();
    marker.pose.position.z = fiducial_position.z() + 0.3;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.text = std::to_string(tag_pair.first);
    fiducial_markers.markers.push_back(marker);
  }

  fiducial_markers_publisher_.publish(fiducial_markers);
}

}  // namespace simple_fiducial_mapping
