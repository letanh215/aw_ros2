#include "control_node.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;

using namespace std::placeholders;

ControlNode::ControlNode()
: rclcpp::Node("control_node"),
selected_speed_(0.0), selected_angular_(0.0),
joy_speed_(0.0), joy_angular_(0.0),
nn_speed_(0.0), nn_angular_(0.0), deadman(false),
nav_mode_(NavMode::JOY)
{
  joy_twist_sub_ = this->create_subscription<Twist>("joy_cmd_vel", 10,
    std::bind(&ControlNode::joy_vel_callback, this, _1));
  nn_twist_sub_ = this->create_subscription<Twist>("nn_cmd_vel", 10,
    std::bind(&ControlNode::nn_vel_callback, this, _1));
  joy_sub_ = this->create_subscription<Joy>("joy", 10,
    std::bind(&ControlNode::joy_callback, this, _1));

  cmd_pub_ = this->create_publisher<Twist>("cmd_vel", 10);

  vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&ControlNode::timer_cmd_vel, this));
  RCLCPP_INFO(this->get_logger(), "Unity Control Node has Started.");
  RCLCPP_INFO(this->get_logger(), "Press X while holding L2/R2 for NN mode and O for Manual(Joy) Mode");
}

void
ControlNode::joy_vel_callback(Twist::SharedPtr msg)
{
  joy_speed_ = msg->linear.x;
  joy_angular_ = msg->angular.z;
}

void
ControlNode::nn_vel_callback(Twist::SharedPtr msg)
{
  nn_speed_ = msg->linear.x;
  nn_angular_ = msg->angular.z;
}

void
ControlNode::joy_callback(Joy::SharedPtr msg)
{
  deadman = (msg->buttons[6] || msg->buttons[7]) ? true : false;

  if (msg->buttons[1]){
    nav_mode_ = NavMode::JOY;
  } else if (msg->buttons[0]) {
    nav_mode_ = NavMode::NN;
  }
}

void
ControlNode::timer_cmd_vel()
{
  if (deadman){
    switch (nav_mode_){
    
      case NavMode::JOY :
        selected_speed_ = joy_speed_;
        selected_angular_ = joy_angular_;
        break;

      case NavMode::NN :
        selected_speed_ = nn_speed_;
        selected_angular_ = nn_angular_;
        break;

      default:
        selected_speed_ = 0.0;
        selected_angular_ = 0.0;
        break;
    }
  } else {
    nav_mode_ = NavMode::JOY;
    selected_speed_ = 0.0;
    selected_angular_ = 0.0;
  }

  Twist msg = Twist();
  msg.linear.x = selected_speed_;
  msg.angular.z = selected_angular_;

  cmd_pub_->publish(msg);
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}