#include <map>
#include <set>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>  // For writing images to disk.
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>

#include <simple_fiducial_mapping/mapping.h>
#include <simple_fiducial_mapping/cv_utils.h>
#include <simple_fiducial_mapping/localization_node.h>
#include <simple_fiducial_mapping/visualizer.h>

using namespace simple_fiducial_mapping;

const std::string tf_topic = "tf";
const std::string tf_static_topic = "tf_static";
const std::string tag_detections_topic = "camera/tag_detections";
const std::string image_topic = "camera/image_rect_downsampled";
const std::string camera_info_topic = "camera/camera_info";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize_from_bag");
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  LocalizationNode localization_node(node_handle, private_node_handle, tf_buffer);
  Visualizer visualizer(node_handle);

  std::string bag_file;
  private_node_handle.param<std::string>("bag_file", bag_file, "");
  if (bag_file == "")
  {
    ROS_FATAL_STREAM("No bagfile specified!  Please give the 'bag_file' parameter the path to a bagfile containing a "
                     "dataset of detected fiducial poses.");
    return -1;
  }

  rosbag::Bag bag;
  try
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagIOException e)
  {
    ROS_FATAL_STREAM("Unable to open bagfile at " << bag_file << "!  "
                                                  << "Did you specify a valid absolute path? \n"
                                                  << e.what());
    return -1;
  }
  ROS_INFO_STREAM("Reading frames from " << bag_file);

  /* Load each message and feed it into the localization node */
  std::vector<std::string> query_topics{ tf_topic, tf_static_topic, tag_detections_topic };
  rosbag::View view(bag, rosbag::TopicQuery(query_topics));
  size_t frame_id = 0;
  std::map<size_t, Eigen::Isometry3d> camera_pose_map;
  for (const auto& message : view)
  {
    if (message.getTopic() == tag_detections_topic)
    {
      const auto& detection_array = message.instantiate<apriltags2_ros::AprilTagDetectionArray>();

      localization_node.localizeFromTags(detection_array);

      // Publish visualization markers for debugging.
      Eigen::Isometry3d most_recent_pose = localization_node.getMostRecentPose();
      camera_pose_map.emplace(frame_id, most_recent_pose);
      visualizer.publishCameraPoses(camera_pose_map);

      frame_id++;
    }
    else if (message.getTopic() == tf_topic)
    {
      const auto& tf_message = message.instantiate<tf2_msgs::TFMessage>();
      ROS_DEBUG("Updating transforms");
      for (const auto& stamped_transform : tf_message->transforms)
      {
        tf_buffer->setTransform(stamped_transform, "bagfile" /* transform authority */, false /* static */);
      }
    }
    else if (message.getTopic() == tf_static_topic)
    {
      const auto& tf_message = message.instantiate<tf2_msgs::TFMessage>();
      ROS_INFO("Updating static transforms");
      for (const auto& stamped_transform : tf_message->transforms)
      {
        tf_buffer->setTransform(stamped_transform, "bagfile" /* transform authority */, true /* static */);
      }
    }
  }

  /* Wait forever for rviz to get our visualization messages */
  ros::spin();

  return 0;
}
