#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <simple_fiducial_mapping/localization_node.h>

using namespace simple_fiducial_mapping;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_node");
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  ROS_INFO("Starting localization node");
  LocalizationNode localization_node(node_handle, private_node_handle, tf_buffer);

  ros::Subscriber tag_detections_subscriber =
      node_handle.subscribe("tag_detections", 1, &LocalizationNode::localizeFromTags, &localization_node);

  ros::spin();

  return 0;
}
