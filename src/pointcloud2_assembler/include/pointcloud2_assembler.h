#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "pcl_conversions/pcl_conversions.h"

#include <vector>
#include <map>
#include <sstream>
#include <mutex>
#include <cmath>

class PointCloud2Assembler : public rclcpp::Node
{
public:
  PointCloud2Assembler();
  ~PointCloud2Assembler();

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string);
  void buildCloud();

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr scan_topic_group_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::PointCloud2 cloud_tmp;
  pcl::PointCloud<pcl::PointXYZ> merged_cloud;

  /* Params */
  std::vector<std::string> topic_names_;
  std::string base_frame_;
  int frequency_;
  std::mutex mutex_;

  rclcpp::QoS qos_ = rclcpp::QoS(1);
};
