#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <eigen3/Eigen/Core>

using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using namespace std::placeholders;

const float ACCELERATION = 1.0f;
const double MGRS_x = 72733.0;
const double MGRS_y = 5322.0;

class GnssNode: public rclcpp::Node
{
public:
    GnssNode(): Node("odom_pub")
    {
        this->qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        this->qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        this->qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        this->gnss_sub_ = this->create_subscription<PoseStamped>(
            "/gnss/pose", this->qos, std::bind(&GnssNode::gnss_callback, this, _1));
        this->odom_gnss_ = this->create_publisher<Odometry>(
            "/gnss/odom", this->qos);

        // Set up odom constant
        this->odom_msg_.child_frame_id = "base_link";
        this->odom_msg_.header.frame_id = "odom";

        this->odom_msg_.pose.pose.orientation.x = 0.0;
        this->odom_msg_.pose.pose.orientation.y = 0.0;
        this->odom_msg_.pose.pose.orientation.z = 0.0;
        this->odom_msg_.pose.pose.orientation.w = 1.0;
    }

private:

    void gnss_callback(const PoseStamped::SharedPtr gnss_msg){
        
        this->odom_msg_.header.stamp = rclcpp::Clock().now();

        this->odom_msg_.pose.pose.position.x = gnss_msg->pose.position.x - MGRS_x;
        this->odom_msg_.pose.pose.position.y = gnss_msg->pose.position.y - MGRS_y;

        this->odom_gnss_->publish(this->odom_msg_);
    }

    rclcpp::Subscription<PoseStamped>::SharedPtr gnss_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr odom_gnss_;

    rclcpp::QoS qos = rclcpp::QoS(10);
    Odometry odom_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GnssNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}