#pragma once

#include <map>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace simple_fiducial_mapping
{
struct CameraPose
{
  Eigen::Vector3d position = { 0.0, 0.0, 0.0 };
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  /** If true, optimizer will not modify this pose. */
  bool is_constant = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FiducialPosition
{
  Eigen::Vector3d position = { 0.0, 0.0, 0.0 };

  /** If true, optimizer will not modify this fiducial position. */
  bool is_constant = false;
};

struct Observation
{
  size_t camera_pose_id;
  size_t fiducial_id;

  /** Position of the fiducial in the camera coordinate frame. */
  Eigen::Vector3d position = { 0.0, 0.0, 0.0 };
};

struct MapGraph
{
  /** Mapping from observation_id to camera_pose_id and fiducial_id for that observation. */
  std::map<size_t, Observation> observations;

  /** Estimated posiiton of each fiducial in the map coordinate frame. */
  std::map<size_t, FiducialPosition> fiducial_positions;

  /** Estimated camera poses in the map coordinate frame. */
  std::map<size_t, CameraPose> camera_poses;
};

struct OptimizationInfo
{
  /** Reprojection error for each observation. */
  std::map<size_t, Eigen::Vector3d> residuals;

  /** Total computation time in seconds. */
  double computation_time;
};

/** Add an observation to the graph; return its unique ID. */
size_t addObservation(size_t camera_pose_id, size_t fiducial_id, const Eigen::Vector3d& position, MapGraph* map_graph);

/** Add a fiducial with the given ID to the graph. */
void addFiducial(size_t fiducial_id, const FiducialPosition& fiducial_position, MapGraph* map_graph);

/** Add a camera pose to the graph; return its unique ID. You can optionally supply the camera_pose_id; otherwise
 * a unique one will be chosen for you.
 */
size_t addCameraPose(size_t camera_pose_id, const CameraPose& camera_pose, MapGraph* map_graph);

/** Remove a camera pose and any observations involving it from the graph. */
void removeCameraPose(size_t camera_pose_id, MapGraph* map_graph);

/** Combine the position and orientiation for this camera pose into one transform */
Eigen::Isometry3d computeCameraTransform(const CameraPose& camera_pose);

double computeCartesianMeanSquareError(const OptimizationInfo& optimization_info);

/** Load fiducial positions from a YAML file. */
bool loadMapYaml(const std::string& map_filename, MapGraph* map_graph);

/** Save fiducial positions to a YAML file. */
bool saveMapYaml(const MapGraph& map_graph, const std::string& filename);

/** Write out the optimization info to a file as YAML. */
bool saveOptimizationInfoYaml(const OptimizationInfo& optimization_info, const std::string& filename);

}
