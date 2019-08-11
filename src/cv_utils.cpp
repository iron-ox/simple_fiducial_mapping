#include <simple_fiducial_mapping/cv_utils.h>
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

namespace simple_fiducial_mapping
{
void draw2DFeatureLocation(const cv::Point2d& image_point, cv::Mat& image)
{
  cv::circle(image, image_point, 5, CV_RGB(0, 0, 255));
}

void drawObservation(size_t fiducial_id, const Eigen::Vector3d fiducial_position,
                     const image_geometry::PinholeCameraModel& camera_model, cv::Mat& image)
{
  // Project the 3d observation point into the image.
  cv::Point3d cv_point(fiducial_position.x(), fiducial_position.y(), fiducial_position.z());
  cv::Point2d image_point = camera_model.project3dToPixel(cv_point);
  cv::circle(image, image_point, 10, CV_RGB(255, 0, 0));

  cv::Point2d text_origin = image_point;
  text_origin.x += 10;

  // Write the fiducial ID next to it on the image.
  char fiducial_label[4];
  snprintf(fiducial_label, 4, "%zu", fiducial_id);
  cv::putText(image, fiducial_label, text_origin, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1,
              CV_AA);
}

void drawObservationsInImage(const std::map<size_t, Eigen::Vector3d>& observations,
                             const image_geometry::PinholeCameraModel& camera_model, cv::Mat& image)
{
  for (const auto& observation_pair : observations)
  {
    drawObservation(observation_pair.first, observation_pair.second, camera_model, image);
  }
}

void drawFiducialPositionsInImage(const Eigen::Isometry3d& camera_pose,
                                  const image_geometry::PinholeCameraModel& camera_model,
                                  const std::map<size_t, Eigen::Vector3d>& fiducial_positions, cv::Mat& image)
{
  Eigen::Isometry3d camera_H_map = camera_pose.inverse();
  for (const auto& fiducial_pair : fiducial_positions)
  {
    Eigen::Vector3d tag_in_map_frame = fiducial_pair.second;
    Eigen::Vector3d tag_in_camera_frame = camera_H_map * tag_in_map_frame;
    cv::Point2d image_point = camera_model.project3dToPixel(
        cv::Point3d(tag_in_camera_frame.x(), tag_in_camera_frame.y(), tag_in_camera_frame.z()));
    draw2DFeatureLocation(image_point, image);
  }
}

void drawTagDetectionsInImage(const std::vector<TagDetection>& tag_detections, cv::Mat& image)
{
  for (const auto& tag_detection : tag_detections)
    draw2DFeatureLocation(tag_detection.center, image);
}

std::vector<TagDetection> findTagsInImage(const cv::Mat& image)
{
  cv::Mat grayscale_image;
  cv::cvtColor(image, grayscale_image, CV_BGR2GRAY);
  image_u8_t raw_image = {.width = grayscale_image.cols,
                          .height = grayscale_image.rows,
                          .stride = grayscale_image.cols,
                          .buf = grayscale_image.data };

  apriltag_family_t* tag_family = tag36h11_create();
  apriltag_detector_t* tag_detector = apriltag_detector_create();
  apriltag_detector_add_family(tag_detector, tag_family);
  tag_detector->refine_decode = 1;
  tag_detector->refine_edges = 1;
  tag_detector->quad_decimate = 1.0;
  tag_detector->qtp.max_nmaxima = 10;
  tag_detector->qtp.min_cluster_pixels = 5;
  tag_detector->qtp.max_line_fit_mse = 10.0;
  tag_detector->qtp.critical_rad = 10 * M_PI / 180;
  tag_detector->qtp.deglitch = 0;
  tag_detector->qtp.min_white_black_diff = 5;
  zarray_t* tag_detections = apriltag_detector_detect(tag_detector, &raw_image);
  std::vector<TagDetection> detections_vector;
  for (int detection_i = 0; detection_i < zarray_size(tag_detections); detection_i++)
  {
    apriltag_detection_t* tag_detection;
    zarray_get(tag_detections, detection_i, &tag_detection);
    detections_vector.push_back(
        TagDetection{.tag_id = tag_detection->id, .center = cv::Point2d(tag_detection->c[0], tag_detection->c[1]) });
  }

  return detections_vector;
}
};
