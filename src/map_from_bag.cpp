#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>

#include <simple_fiducial_mapping/ceres_backend.h>
#include <simple_fiducial_mapping/cv_utils.h>
#include <simple_fiducial_mapping/mapping.h>
#include <simple_fiducial_mapping/map_graph.h>
#include <simple_fiducial_mapping/visualizer.h>

using namespace simple_fiducial_mapping;

const std::string tag_detections_topic = "camera/tag_detections";
const std::string image_topic = "camera/image_rect_downsampled";
const std::string camera_info_topic = "camera/camera_info";
const std::string map_filename = "map.yaml";
const std::string output_bag_path = "mapping_log.bag";

int main(int argc, char** argv)
{
  // Needed before calling any Ceres functions.
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "map_from_bag");
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");

  // Only use 1 out of every subsample_step camera poses.
  int subsample_step;
  private_node_handle.param<int>("subsample_step", subsample_step, 10);

  // Don't keep new camera poses unless we've moved at least this far. Not perfect, since we only know
  // how far we've moved by running the optimizer, and there is always error. Still a good heuristic to keep
  // the graph from getting too dense and taking too long to optimize.
  double min_step_distance;
  private_node_handle.param<double>("min_step_distance", min_step_distance, 1.0);

  // Skip N tag detection messages. Useful for debugging bad frames later in a bag file.
  int start_index;
  private_node_handle.param<int>("start_index", start_index, 0);

  // Last tag detection message index to use. We process [start_index, end_index] frames. (Includes start and end
  // indices)
  int end_index;
  private_node_handle.param<int>("end_index", end_index, INT_MAX);

  // Maximum number of camera poses to add to the graph. Because we skip ones that are too close together, this
  // isn't the same as end_index-start_index.
  int max_poses;
  private_node_handle.param<int>("max_poses", max_poses, INT_MAX);

  bool save_frame_images;
  private_node_handle.param<bool>("save_frame_images", save_frame_images, false);
  ROS_INFO("Saving of images for frames enabled");

  // Each time we add a new pose and set of observations, we recompute the RMSE. If it exceeds this threshold, we will
  // throw out the frame and keep going.
  double max_rmse;
  private_node_handle.param<double>("max_rmse", max_rmse, 1.0);

  // If true, only use frames that have an image in the bagfile. Useful for debugging.
  bool images_required;
  private_node_handle.param<bool>("images_required", images_required, false);

  std::string input_bag_path;
  private_node_handle.param<std::string>("bag_file", input_bag_path, "");
  if (input_bag_path == "")
  {
    ROS_FATAL_STREAM("No bagfile specified! Set bag_file param to bag path");
    return 0;
  }

  rosbag::Bag input_bag;
  try
  {
    input_bag.open(input_bag_path, rosbag::bagmode::Read);
  }
  catch (rosbag::BagIOException e)
  {
    ROS_FATAL_STREAM("Unable to open input bagfile at " << input_bag_path);
    return 0;
  }
  ROS_INFO_STREAM("Reading input data from " << input_bag_path);
  rosbag::View view(input_bag, rosbag::TopicQuery(std::vector<std::string>{ tag_detections_topic }));

  rosbag::Bag output_bag;
  try
  {
    output_bag.open(output_bag_path, rosbag::bagmode::Write);
  }
  catch (rosbag::BagIOException e)
  {
    ROS_FATAL_STREAM("Unable to open output bagfile at " << output_bag_path);
    return 0;
  }
  ROS_INFO_STREAM("Writing log data to " << output_bag_path);

  MapGraph map_graph;
  Visualizer visualizer(node_handle);
  int detection_i = -1;
  std::vector<size_t> pose_id_sequence;
  for (const auto& message : view)
  {
    if (!ros::ok())
    {
      ROS_INFO("Shutdown signal received");
      break;
    }
    detection_i += 1;
    if (pose_id_sequence.size() >= max_poses)
    {
      ROS_INFO("Reached maximum number of poses (%d), stopping", max_poses);
      break;
    }
    if (detection_i > end_index)
    {
      ROS_INFO("Reached end index %d, stopping", detection_i);
      break;
    }
    if (detection_i < start_index)
    {
      continue;
    }
    if ((detection_i % subsample_step) != 0)
    {
      continue;
    }

    const auto detections_msg = message.instantiate<apriltags2_ros::AprilTagDetectionArray>();
    if (detections_msg->detections.size() < 5)
    {
      ROS_ERROR_STREAM("Skipping camera pose with " << detections_msg->detections.size() << " detections");
      continue;
    }

    // Add the new camera pose and observations to the graph. We manually set the camera pose id to detection_i so
    // that we know which detetections message in the bagfile this pose corresponds to.
    size_t new_camera_pose_id = addCameraPose(detection_i, CameraPose{}, &map_graph);
    std::map<size_t, Eigen::Vector3d> observations = tagDetectionsToObservations(*detections_msg);
    for (const auto& fiducial_pair : observations)
    {
      addObservation(new_camera_pose_id, fiducial_pair.first, fiducial_pair.second, &map_graph);
    }

    // If it looks like we haven't moved far, remove this pose from the graph.
    if (pose_id_sequence.size() > 0)
    {
      // Start the current pose at the previous pose
      map_graph.camera_poses[new_camera_pose_id].position = map_graph.camera_poses[pose_id_sequence.back()].position;
      map_graph.camera_poses[new_camera_pose_id].orientation =
          map_graph.camera_poses[pose_id_sequence.back()].orientation;

      // Fix all the fiducials and camera poses so that we're just localizing the new pose. We'll use this to detect how
      // far we've moved.
      for (auto& fiducial_position_pair : map_graph.fiducial_positions)
      {
        map_graph.fiducial_positions.at(fiducial_position_pair.first).is_constant = true;
      }
      for (auto& camera_pose_pair : map_graph.camera_poses)
      {
        map_graph.camera_poses.at(camera_pose_pair.first).is_constant = true;
      }
      map_graph.camera_poses.at(new_camera_pose_id).is_constant = false;

      OptimizationInfo pose_check_optimization_info;
      pose_check_optimization_info = bundleAdjust(3, &map_graph);

      double root_mean_square_error = sqrt(computeCartesianMeanSquareError(pose_check_optimization_info));
      const Eigen::Isometry3d& current_pose = computeCameraTransform(map_graph.camera_poses.at(new_camera_pose_id));
      const Eigen::Isometry3d& previous_pose =
          computeCameraTransform(map_graph.camera_poses.at(pose_id_sequence.back()));
      double distance_traveled = (previous_pose.translation() - current_pose.translation()).norm();
      ROS_INFO("Computed pose check in %f seconds, average residual is %f distance traveled is %f",
               pose_check_optimization_info.computation_time, root_mean_square_error, distance_traveled);
      if (root_mean_square_error > max_rmse)
      {
        ROS_INFO("Skipping frame %d because RMSE is too high", detection_i);
        removeCameraPose(new_camera_pose_id, &map_graph);
        continue;
      }
      else if (distance_traveled < min_step_distance)
      {
        ROS_INFO("Skipping frame %d because we didn't move far enough", detection_i);
        removeCameraPose(new_camera_pose_id, &map_graph);
        continue;
      }

      // Set all camera poses as variable again, except for the very first.
      for (auto& fiducial_position_pair : map_graph.fiducial_positions)
      {
        map_graph.fiducial_positions.at(fiducial_position_pair.first).is_constant = false;
      }
      for (auto& camera_pose_pair : map_graph.camera_poses)
      {
        map_graph.camera_poses.at(camera_pose_pair.first).is_constant = false;
      }
      map_graph.camera_poses.at(pose_id_sequence.front()).is_constant = true;
    }
    pose_id_sequence.push_back(new_camera_pose_id);

    OptimizationInfo graph_update_optimization_info;
    graph_update_optimization_info = bundleAdjust(10, &map_graph);

    double root_mean_square_error = sqrt(computeCartesianMeanSquareError(graph_update_optimization_info));
    ROS_INFO("%d: %zu residuals for %zu camera poses in %f seconds, rmse: %f", detection_i,
             graph_update_optimization_info.residuals.size(), map_graph.camera_poses.size(),
             graph_update_optimization_info.computation_time, root_mean_square_error);

    visualizer.publishGraphVisualization(map_graph);
  }

  input_bag.close();

  if (saveMapYaml(map_graph, map_filename))
  {
    ROS_INFO_STREAM("Saved map to " << map_filename);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to save map to " << map_filename);
  }

  ROS_INFO("Published visualization messages; spinning.");
  ros::spin();

  return 0;
}
