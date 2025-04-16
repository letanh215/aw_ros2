#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <eigen3/Eigen/Core>

using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using namespace std::placeholders;

const float ACCELERATION = 1.0f;
const double MGRS_x = 72733.0;
const double MGRS_y = 5322.0;

class OdomUpdate: public rclcpp::Node
{
public:
    OdomUpdate(): Node("odom_update_node")
    {
        qos_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        this->firstOdom = true;

        this->pose_sub_ = this->create_subscription<PoseStamped>(
            "/awsim/ground_truth/vehicle/pose", qos_, std::bind(&OdomUpdate::pose_callback, this, _1));
        this->tfb_ = std::make_shared<TransformBroadcaster>(*this);

        this->odom_msg_.header.frame_id = "odom";
        this->odom_msg_.child_frame_id = "base_link";

        // this->map_msgs_.header.frame_id = "map";
        // this->map_msgs_.child_frame_id = "odom";
        // this->map_msgs_.transform.translation.x = 0.0;
        // this->map_msgs_.transform.translation.y = 0.0;
        // this->map_msgs_.transform.translation.z = 0.0;
        // this->map_msgs_.transform.rotation.x = 0.0;
        // this->map_msgs_.transform.rotation.y = 0.0;
        // this->map_msgs_.transform.rotation.z = 0.0;
        // this->map_msgs_.transform.rotation.w = 1.0;

        this->tfb_->sendTransform(this->map_msgs_);

        RCLCPP_INFO_STREAM(this->get_logger(), "Broadcasting odom-base_link transform.");
    }

private:

    void pose_callback(const PoseStamped::SharedPtr msg){
        odom_msg_.header.stamp = rclcpp::Clock().now();
        if (this->firstOdom){
            this->first_odom_= msg->pose;
            this->firstOdom = false;
            this->odom_msg_.transform.translation.x = 0.0;
            this->odom_msg_.transform.translation.y = 0.0;
            this->odom_msg_.transform.translation.z = 0.0;

            this->odom_msg_.transform.rotation = msg->pose.orientation;
        }


        odom_msg_.transform.translation.x = msg->pose.position.x - this->first_odom_.position.x;
        // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << odom_msg_.transform.translation.x);
        odom_msg_.transform.translation.y = msg->pose.position.y - this->first_odom_.position.y;
        odom_msg_.transform.translation.z = 0;

        odom_msg_.transform.rotation.x = msg->pose.orientation.x;
        odom_msg_.transform.rotation.y = msg->pose.orientation.y;
        odom_msg_.transform.rotation.z = msg->pose.orientation.z;
        odom_msg_.transform.rotation.w = msg->pose.orientation.w;

        this->tfb_->sendTransform(this->odom_msg_);
    }

    rclcpp::QoS qos_ = rclcpp::QoS(10);
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;

    TransformStamped odom_msg_;
    TransformStamped map_msgs_;
    std::shared_ptr<TransformBroadcaster> tfb_;

    Pose first_odom_;
    bool firstOdom;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomUpdate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}