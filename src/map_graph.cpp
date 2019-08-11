#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <simple_fiducial_mapping/map_graph.h>

namespace simple_fiducial_mapping
{
size_t addObservation(size_t camera_pose_id, size_t fiducial_id, const Eigen::Vector3d& position, MapGraph* map_graph)
{
  // You must add the camera pose before adding the observation.
  assert(map_graph->camera_poses.count(camera_pose_id) != 0);

  // New ID is one larger than largest existing ID.
  size_t observation_id;
  if (map_graph->observations.empty())
  {
    observation_id = 0;
  }
  else
  {
    observation_id = map_graph->observations.crbegin()->first + 1;
  }

  // In camera frame, negative z would be behind the camera, which would make no sense. Z=0 would cause a divide by
  // zero when we project the point into the image.
  assert(position.z() > 0.0);

  if (map_graph->fiducial_positions.count(fiducial_id) == 0)
  {
    // Create a new fiducial position, with an initial pose based on the camera position and observed fiducial position
    // in the camera coordinate frame.
    const Eigen::Isometry3d& camera_pose = computeCameraTransform(map_graph->camera_poses.at(camera_pose_id));
    Eigen::Vector3d fiducial_initial_position = camera_pose * position;
    map_graph->fiducial_positions.emplace(fiducial_id, FiducialPosition{ fiducial_initial_position });
  }

  map_graph->observations.emplace(observation_id, Observation{ camera_pose_id, fiducial_id, position });

  return observation_id;
}

void removeObservation(size_t observation_id, MapGraph* map_graph)
{
  assert(map_graph->observations.count(observation_id) > 0);
  map_graph->observations.erase(observation_id);
}

size_t addCameraPose(size_t camera_pose_id, const CameraPose& camera_pose, MapGraph* map_graph)
{
  if (camera_pose_id == SIZE_MAX)
  {
    // No pose id supplied; create a unique one. New ID is one larger than largest existing ID.
    if (map_graph->camera_poses.empty())
    {
      camera_pose_id = 0;
    }
    else
    {
      camera_pose_id = map_graph->camera_poses.crbegin()->first + 1;
    }
  }

  map_graph->camera_poses[camera_pose_id] = camera_pose;

  return camera_pose_id;
}

void addFiducial(size_t fiducial_id, const FiducialPosition& fiducial_position, MapGraph* map_graph)
{
  if (map_graph->fiducial_positions.count(fiducial_id) == 0)
  {
    map_graph->fiducial_positions.emplace(fiducial_id, fiducial_position);
  }
}

void removeCameraPose(size_t camera_pose_id, MapGraph* map_graph)
{
  for (auto observation_iter = map_graph->observations.cbegin(); observation_iter != map_graph->observations.cend();)
  {
    size_t observation_id = observation_iter->first;
    if (observation_iter->second.camera_pose_id == camera_pose_id)
    {
      map_graph->observations.erase(observation_iter++);
    }
    else
    {
      observation_iter++;
    }
  }
  map_graph->camera_poses.erase(camera_pose_id);
}

Eigen::Isometry3d computeCameraTransform(const CameraPose& camera_pose)
{
  return Eigen::Translation3d(camera_pose.position) * camera_pose.orientation;
}

double computeCartesianMeanSquareError(const OptimizationInfo& optimization_info)
{
  double mean_square_error = 0;
  for (const auto& residual : optimization_info.residuals)
  {
    mean_square_error += residual.second.x() * residual.second.x();
    mean_square_error += residual.second.y() * residual.second.y();
    mean_square_error += residual.second.z() * residual.second.z();
  }
  mean_square_error /= (optimization_info.residuals.size() * 3.);
  return mean_square_error;
}

bool saveMapYaml(const MapGraph& map_graph, const std::string& filename)
{
  YAML::Emitter map_yaml;
  map_yaml << YAML::BeginMap;
  for (auto& fiducial_pair : map_graph.fiducial_positions)
  {
    map_yaml << YAML::Key << fiducial_pair.first;
    map_yaml << YAML::Value << YAML::Flow << YAML::BeginSeq;
    Eigen::Vector3d fiducial_position = fiducial_pair.second.position;
    map_yaml << fiducial_position.x() << fiducial_position.y() << fiducial_position.z();
    map_yaml << YAML::EndSeq;
  }
  map_yaml << YAML::EndMap;

  std::ofstream map_ofstream(filename);
  if (!map_ofstream.is_open())
    return false;

  map_ofstream << map_yaml.c_str();

  return true;
}

bool loadMapYaml(const std::string& map_filename, MapGraph* map_graph)
{
  try
  {
    YAML::Node map_yaml = YAML::LoadFile(map_filename);
    auto fiducials_map = map_yaml.as<std::map<size_t, std::vector<double>>>();
    for (auto& fiducial_pair : fiducials_map)
    {
      size_t fiducial_id = fiducial_pair.first;
      Eigen::Vector3d fiducial_position(fiducial_pair.second[0], fiducial_pair.second[1], fiducial_pair.second[2]);
      assert(fiducial_pair.second.size() >= 3);

      addFiducial(fiducial_id, FiducialPosition{ fiducial_position, true /* is_constant */ }, map_graph);
    }
  }
  catch (YAML::BadFile)
  {
    ROS_ERROR("Unable to load map from %s", map_filename.c_str());
    return false;
  }
  return true;
}

bool saveOptimizationInfoYaml(const OptimizationInfo& optimization_info, const std::string& filename)
{
  YAML::Emitter optimization_info_yaml;
  optimization_info_yaml << YAML::BeginMap;
  for (auto& residual_pair : optimization_info.residuals)
  {
    optimization_info_yaml << YAML::Key << residual_pair.first;
    optimization_info_yaml << YAML::Value << YAML::Flow << YAML::BeginSeq;
    Eigen::Vector3d residual_value = residual_pair.second;
    optimization_info_yaml << residual_value.x() << residual_value.y() << residual_value.z();
    optimization_info_yaml << YAML::EndSeq;
  }
  optimization_info_yaml << YAML::EndMap;

  std::ofstream map_ofstream(filename);
  if (!map_ofstream.is_open())
    return false;

  map_ofstream << optimization_info_yaml.c_str();

  return true;
}

}
