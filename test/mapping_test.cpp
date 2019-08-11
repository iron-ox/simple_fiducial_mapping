#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <simple_fiducial_mapping/ceres_backend.h>
#include <simple_fiducial_mapping/mapping.h>
#include <simple_fiducial_mapping/map_graph.h>

#define DOUBLE_ERROR_TOLERANCE 0.0001

using namespace simple_fiducial_mapping;

std::map<size_t, Eigen::Vector3d> create_simulated_fiducial_observations(
    const Eigen::Isometry3d& world_H_camera, const std::map<size_t, Eigen::Vector3d>& actual_fiducial_positions)
{
  std::map<size_t, Eigen::Vector3d> simulated_fiducial_observations;

  Eigen::Isometry3d camera_H_world = world_H_camera.inverse();

  for (const auto& fiducial_pair : actual_fiducial_positions)
  {
    size_t fiducial_id = fiducial_pair.first;
    const Eigen::Vector3d& world_frame_fiducial_position = fiducial_pair.second;
    const Eigen::Vector3d camera_frame_fiducial_position = camera_H_world * world_frame_fiducial_position;
    simulated_fiducial_observations[fiducial_id] = camera_frame_fiducial_position;
  }

  return simulated_fiducial_observations;
}

TEST(MapppingTest, cartesianError)
{
  std::vector<Eigen::Isometry3d> actual_camera_poses;
  actual_camera_poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0)));
  actual_camera_poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0)));

  std::map<size_t, Eigen::Vector3d> actual_fiducial_positions;
  actual_fiducial_positions[0] = Eigen::Vector3d(0.0, 0.0, 3.0);
  actual_fiducial_positions[1] = Eigen::Vector3d(1.0, 0.0, 3.0);
  actual_fiducial_positions[2] = Eigen::Vector3d(0.0, 1.0, 3.0);

  for (const auto& camera_pose : actual_camera_poses)
  {
    std::map<size_t, Eigen::Vector3d> observations =
        create_simulated_fiducial_observations(camera_pose, actual_fiducial_positions);
    Eigen::Vector3d camera_position = camera_pose.translation();
    Eigen::Quaterniond camera_orientation(camera_pose.rotation());
    for (const auto& observation_pair : observations)
    {
      Eigen::Vector3d& actual_fiducial_position_wf = actual_fiducial_positions.at(observation_pair.first);
      const Eigen::Vector3d& observed_fiducial_position_cf = observation_pair.second;
      CartesianCostFunction cost_function(observed_fiducial_position_cf);
      Eigen::Vector3d residuals;
      EXPECT_TRUE(cost_function(camera_orientation.coeffs().data(), camera_position.data(),
                                actual_fiducial_position_wf.data(), residuals.data()));
      EXPECT_NEAR(residuals[0], 0.0, DOUBLE_ERROR_TOLERANCE);
      EXPECT_NEAR(residuals[1], 0.0, DOUBLE_ERROR_TOLERANCE);
      EXPECT_NEAR(residuals[2], 0.0, DOUBLE_ERROR_TOLERANCE);
    }
  }
}

TEST(MappingTest, makeSimpleMap)
{
  std::vector<Eigen::Isometry3d> actual_camera_poses;
  actual_camera_poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0)));
  actual_camera_poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0)));

  std::map<size_t, Eigen::Vector3d> actual_fiducial_positions;
  actual_fiducial_positions[0] = Eigen::Vector3d(0.0, 0.0, 3.0);
  actual_fiducial_positions[1] = Eigen::Vector3d(1.0, 0.0, 3.0);
  actual_fiducial_positions[2] = Eigen::Vector3d(0.0, 1.0, 3.0);

  MapGraph map_graph;
  for (size_t pose_i = 0; pose_i < actual_camera_poses.size(); pose_i++)
  {
    const auto& world_H_camera = actual_camera_poses[pose_i];
    std::map<size_t, Eigen::Vector3d> simulated_fiducial_observations =
        create_simulated_fiducial_observations(world_H_camera, actual_fiducial_positions);

    size_t camera_pose_id = addCameraPose(SIZE_MAX, CameraPose{ Eigen::Vector3d{ 0.0, 0.0, 0.0 } }, &map_graph);
    for (const auto& observation_pair : simulated_fiducial_observations)
    {
      addObservation(camera_pose_id, observation_pair.first, observation_pair.second, &map_graph);
    }

    // Fix first camera pose to arbitrarily locate the map.
    if (pose_i == 0)
    {
      map_graph.camera_poses.at(camera_pose_id).position = world_H_camera.translation();
      map_graph.camera_poses.at(camera_pose_id).orientation = world_H_camera.rotation();
      map_graph.camera_poses.at(camera_pose_id).is_constant = true;
    }
    bundleAdjust(1, &map_graph);
  }
  bundleAdjust(200, &map_graph);

  for (const auto& fiducial_pair : actual_fiducial_positions)
  {
    size_t fiducial_id = fiducial_pair.first;
    const Eigen::Vector3d& actual_fiducial_position = fiducial_pair.second;
    const Eigen::Vector3d estimated_fiducial_position = map_graph.fiducial_positions.at(fiducial_id).position;

    EXPECT_NEAR(estimated_fiducial_position.x(), actual_fiducial_position.x(), DOUBLE_ERROR_TOLERANCE);
    EXPECT_NEAR(estimated_fiducial_position.y(), actual_fiducial_position.y(), DOUBLE_ERROR_TOLERANCE);
    EXPECT_NEAR(estimated_fiducial_position.z(), actual_fiducial_position.z(), DOUBLE_ERROR_TOLERANCE);
  }
}

TEST(LocalizationTest, localizesInSimpleMap)
{
  Eigen::Isometry3d actual_camera_pose = Eigen::Isometry3d(Eigen::Translation3d(0.3, 0.1, 0.0));

  std::map<size_t, Eigen::Vector3d> actual_fiducial_positions;
  actual_fiducial_positions[0] = Eigen::Vector3d(0.0, 0.0, 3.0);
  actual_fiducial_positions[1] = Eigen::Vector3d(1.0, 0.0, 3.0);
  actual_fiducial_positions[2] = Eigen::Vector3d(0.0, 1.0, 3.0);
  actual_fiducial_positions[3] = Eigen::Vector3d(0.2, 1.0, 3.0);

  MapGraph map_graph;
  for (const auto& fiducial_pair : actual_fiducial_positions)
  {
    size_t fiducial_id = fiducial_pair.first;
    const Eigen::Vector3d& fiducial_position = fiducial_pair.second;
    addFiducial(fiducial_id, FiducialPosition{ fiducial_position }, &map_graph);
    map_graph.fiducial_positions.at(fiducial_pair.first).is_constant = true;
  }

  std::map<size_t, Eigen::Vector3d> simulated_fiducial_observations =
      create_simulated_fiducial_observations(actual_camera_pose, actual_fiducial_positions);

  // Add a bogus fiducial detection (the localization code should ignore it)
  simulated_fiducial_observations[999] = Eigen::Vector3d(1.0, -282.2, 99.234);

  Eigen::Isometry3d computed_camera_pose;

  EXPECT_TRUE(localize(simulated_fiducial_observations, Eigen::Isometry3d::Identity(),
                       std::numeric_limits<double>::infinity(), &computed_camera_pose, &map_graph));

  EXPECT_NEAR(computed_camera_pose.translation().x(), actual_camera_pose.translation().x(), 0.0001);
  EXPECT_NEAR(computed_camera_pose.translation().y(), actual_camera_pose.translation().y(), 0.0001);
  EXPECT_NEAR(computed_camera_pose.translation().z(), actual_camera_pose.translation().z(), 0.0001);

  Eigen::Isometry3d error_transform = computed_camera_pose.inverse() * actual_camera_pose;
  Eigen::AngleAxis<double> angle_error(error_transform.rotation());
  EXPECT_NEAR(angle_error.angle(), 0.0, 0.0001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
