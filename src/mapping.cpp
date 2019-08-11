#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <simple_fiducial_mapping/mapping.h>
#include <simple_fiducial_mapping/ceres_backend.h>

namespace simple_fiducial_mapping
{
bool areObservationsGoodForSlam(const std::map<size_t, Eigen::Vector3d>& observations)
{
  if (observations.size() < 3)
  {
    ROS_ERROR_STREAM("We got less than 3 observations.");
    return false;
  }

  // Project observations onto ideal image plane.
  std::vector<Eigen::Vector2d> ideal_image_plane_points;
  for (const auto& observation_pair : observations)
  {
    const Eigen::Vector3d& position = observation_pair.second;
    if (position.z() >= OBSERVATION_MIN_DISTANCE)
      // Throw out any observations too close to the camera (they have high error after reprojection).
      ideal_image_plane_points.push_back(Eigen::Vector2d(position.x() / position.z(), position.y() / position.z()));
  }

  if (ideal_image_plane_points.size() < 3)
  {
    ROS_ERROR_STREAM("The number of ideal image plane points is less than 3.");
    return false;
  }

  // Unit vectors along which we'll check point spread.
  const std::vector<Eigen::Vector2d> direction_vectors{
    { cos(0.0), sin(0.0) }, { cos(M_PI / 4.0), sin(M_PI / 4.0) }, { cos(M_PI / 2.0), sin(M_PI / 2.0) },
  };

  // Spread must be large enough along all of our direction vectors.
  for (const auto direction : direction_vectors)
  {
    double min_val = direction.dot(ideal_image_plane_points[0]);
    double max_val = direction.dot(ideal_image_plane_points[0]);
    for (const auto& point : ideal_image_plane_points)
    {
      double dot_val = direction.dot(point);
      if (dot_val < min_val)
        min_val = dot_val;
      if (dot_val > max_val)
        max_val = dot_val;
    }
    double spread = max_val - min_val;
    // Intentionally written as a negative check so that a spread of NaN will return false.
    if (!(spread > OBSERVATION_MIN_SPREAD))
    {
      ROS_ERROR_STREAM(
          "The current spread: " << spread << " is less than the observation min spread: " << OBSERVATION_MIN_SPREAD);
      return false;
    }
  }

  return true;
}

std::map<size_t, Eigen::Vector3d>
tagDetectionsToObservations(const apriltags2_ros::AprilTagDetectionArray& detections_msg)
{
  std::map<size_t, Eigen::Vector3d> observations;
  for (const auto& detection : detections_msg.detections)
  {
    assert(detection.id.size() == 1);
    size_t fiducial_id = detection.id[0];
    const auto& position_msg = detection.pose.pose.pose.position;
    observations[fiducial_id] = Eigen::Vector3d(position_msg.x, position_msg.y, position_msg.z);
  }
  return observations;
}

bool localize(const std::map<size_t, Eigen::Vector3d>& observations, const Eigen::Isometry3d& camera_pose_guess,
              double max_rmse, Eigen::Isometry3d* computed_camera_pose, MapGraph* map_graph)
{
  if (!areObservationsGoodForSlam(observations))
  {
    ROS_ERROR("Observations are not spread out; not localizing");
    return false;
  }

  Eigen::Quaterniond orientation_guess(camera_pose_guess.rotation());
  Eigen::Vector3d position_guess = camera_pose_guess.translation();

  ROS_DEBUG_STREAM("Localizing from " << observations.size() << " observations");

  size_t camera_pose_id =
      addCameraPose(0, CameraPose{ position_guess, orientation_guess, false /* is_constant */ }, map_graph);
  for (const auto& observation_pair : observations)
  {
    size_t fiducial_id = observation_pair.first;
    Eigen::Vector3d fiducial_position = observation_pair.second;
    addObservation(camera_pose_id, fiducial_id, fiducial_position, map_graph);
  }
  OptimizationInfo localization_optimization_info = bundleAdjust(20, map_graph);
  double rmse = sqrt(computeCartesianMeanSquareError(localization_optimization_info));
  ROS_INFO("Localized with RMSE of %f", rmse);
  if (rmse > max_rmse)
  {
    return false;
  }

  const CameraPose& camera_pose = map_graph->camera_poses.at(camera_pose_id);
  *computed_camera_pose = Eigen::Translation3d(camera_pose.position) * camera_pose.orientation;

  removeCameraPose(camera_pose_id, map_graph);

  return true;
}

}  // namespace simple_fiducial_mapping
