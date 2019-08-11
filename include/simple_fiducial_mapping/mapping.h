#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <simple_fiducial_mapping/map_graph.h>

namespace simple_fiducial_mapping
{
/** Observations closer to the camera than this will be ignored, since reprojection of closeup points
 * leads to a lot of error. Expressed in meters.
 */
static const double OBSERVATION_MIN_DISTANCE = 0.5;

/** Minimum distance between furthest apart observation points along any direction in the ideal image plane.
 */
static const double OBSERVATION_MIN_SPREAD = 0.2;

/** Test whether a set of observations in a frame are well defined for slam.
 */
bool areObservationsGoodForSlam(const std::vector<Eigen::Vector3d>& observations);

/** Converts an apriltags detection message into a set of 3D observations.
 */
std::map<size_t, Eigen::Vector3d>
tagDetectionsToObservations(const apriltags2_ros::AprilTagDetectionArray& detections_msg);

/** Solve for camera pose given known fiducial positions in the map coordinate frame and observed fiducial positions in
 * the camera frame.
 *
 * map_graph must have the fiducial positions already added and set to constant.
 */
bool localize(const std::map<size_t, Eigen::Vector3d>& observations, const Eigen::Isometry3d& camera_pose_guess,
              double max_rmse, Eigen::Isometry3d* computed_camera_pose, MapGraph* map_graph);

}  // namespace simple_fiducial_mapping
