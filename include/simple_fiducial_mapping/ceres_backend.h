#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <simple_fiducial_mapping/map_graph.h>

namespace simple_fiducial_mapping
{
/** 2D distance between the observed and estimated feature positions on the ideal image plane */
struct ReprojectionErrorFunction
{
  ReprojectionErrorFunction(const Eigen::Vector3d& observed_position) : observed_position_(observed_position)
  {
  }

  template <typename T>
  bool operator()(const T* const camera_rotation_array, const T* camera_translation_array,
                  const T* const fiducial_position_array, T* residuals) const;

  Eigen::Vector3d observed_position_;
};

/* 3D distance between the observed and estimated feature positions */
struct CartesianCostFunction
{
  CartesianCostFunction(const Eigen::Vector3d& observed_position) : observed_position_(observed_position)
  {
  }

  template <typename T>
  bool operator()(const T* const camera_rotation_array, const T* camera_translation_array,
                  const T* const fiducial_position_array, T* residuals) const;

  const Eigen::Vector3d observed_position_;
};

template <typename T>
bool ReprojectionErrorFunction::operator()(const T* const camera_rotation_array, const T* camera_translation_array,
                                           const T* const fiducial_position_array, T* residuals) const
{
  // Convert the array of doubles Ceres uses for the parameters into Eigen types.
  Eigen::Quaternion<T> camera_rotation = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation_array);
  Eigen::Matrix<T, 3, 1> camera_translation = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(camera_translation_array);
  Eigen::Matrix<T, 3, 1> fiducial_position_wf = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(fiducial_position_array);

  Eigen::Matrix<T, 3, 1> predicted_fiducial_position_cf =
      camera_rotation.inverse() * (fiducial_position_wf - camera_translation);

  // Project estimated points onto ideal image plane.
  T x_ideal_estimated = predicted_fiducial_position_cf[0] / predicted_fiducial_position_cf[2];
  T y_ideal_estimated = predicted_fiducial_position_cf[1] / predicted_fiducial_position_cf[2];

  // Re-projecting the observed point every time is a bit wasteful since it doesnt change, but it keeps the code simple.
  residuals[0] = x_ideal_estimated - observed_position_.x() / observed_position_.z();
  residuals[1] = y_ideal_estimated - observed_position_.y() / observed_position_.z();

  return true;
}

template <typename T>
bool CartesianCostFunction::operator()(const T* const camera_rotation_array, const T* camera_translation_array,
                                       const T* const fiducial_position_array, T* residuals) const
{
  // Convert the array of doubles Ceres uses for the parameters into Eigen types.
  Eigen::Quaternion<T> camera_rotation = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation_array);
  Eigen::Matrix<T, 3, 1> camera_translation = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(camera_translation_array);
  Eigen::Matrix<T, 3, 1> fiducial_position_wf = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(fiducial_position_array);

  Eigen::Matrix<T, 3, 1> predicted_fiducial_position_cf =
      camera_rotation.inverse() * (fiducial_position_wf - camera_translation);

  residuals[0] = predicted_fiducial_position_cf[0] - observed_position_.x();
  residuals[1] = predicted_fiducial_position_cf[1] - observed_position_.y();
  residuals[2] = predicted_fiducial_position_cf[2] - observed_position_.z();

  return true;
}

OptimizationInfo bundleAdjust(int max_iterations, MapGraph* map_graph);
} /* End of namespace simple_fiducial_mapping */
