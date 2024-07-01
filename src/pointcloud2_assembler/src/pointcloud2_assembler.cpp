#include "pointcloud2_assembler.h"

PointCloud2Assembler::PointCloud2Assembler()
: Node("pointcloud2_assembler")
{
  // ***** Set fixed_frame *****
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  topic_names_ = this->declare_parameter("scan_topics", std::vector<std::string>{});
  frequency_ = this->declare_parameter("frequency", 10);

  /* initialise tf */
  tf_.reset(new tf2_ros::Buffer(this->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_));

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_merged", 10);

  this->qos_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  this->qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  this->qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  for (auto topic : topic_names_) {
    RCLCPP_INFO(this->get_logger(), "Subscribed to scans on topic: %s", topic.c_str());
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> fcn = std::bind(
      &PointCloud2Assembler::cloud_callback, this, std::placeholders::_1, topic);

    subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, qos_, fcn));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    clouds_[topic] = cloud_out;
  }

  timer_ =
    rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(
      (int)(1000 / frequency_)), std::bind(&PointCloud2Assembler::buildCloud, this));
}

PointCloud2Assembler::~PointCloud2Assembler() {}

void PointCloud2Assembler::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in,
  const std::string topic)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  try {
    cloud_tmp = tf_->transform(*cloud_in, base_frame_, tf2::durationFromSec(2.0));
    clouds_[topic]->clear();
    pcl::fromROSMsg(cloud_tmp, *clouds_[topic]);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform point.");
  }
}

void PointCloud2Assembler::buildCloud()
{
  const std::lock_guard<std::mutex> lock(mutex_);

  merged_cloud.clear();
  for (auto & pair : clouds_) {
    merged_cloud += *pair.second;
    pair.second->clear();
  }

  if (merged_cloud.empty()) {
    RCLCPP_INFO(this->get_logger(), "No clouds received.");
  } else {
    pcl::toROSMsg(merged_cloud, cloud_tmp);
    cloud_tmp.header.frame_id = base_frame_;
    cloud_tmp.header.stamp = this->now();
    cloud_pub_->publish(cloud_tmp);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PointCloud2Assembler>());
  rclcpp::shutdown();

  return 0;
}
