#pragma once

#include "apriltags2_ros/common_functions.h"
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

#include <simple_fiducial_mapping/mapping.h>

namespace simple_fiducial_mapping
{
struct TagDetection
{
  int tag_id;
  cv::Point2d center;
};

void draw2DFeatureLocation(const cv::Point2d& image_point, cv::Mat& image);

void drawTagDetectionsInImage(const std::vector<TagDetection>& tag_detections, cv::Mat& image);

void drawObservationsInImage(const std::map<size_t, Eigen::Vector3d>& observations,
                             const image_geometry::PinholeCameraModel& camera_model, cv::Mat& image);

void drawFiducialPositionsInImage(const Eigen::Isometry3d& camera_pose,
                                  const image_geometry::PinholeCameraModel& camera_model,
                                  const std::map<size_t, Eigen::Vector3d>& fiducial_positions, cv::Mat& image);

void drawTagDetectionsInImage(const std::vector<TagDetection>& tag_detections, cv::Mat& image);

std::vector<TagDetection> findTagsInImage(const cv::Mat& image);
};
