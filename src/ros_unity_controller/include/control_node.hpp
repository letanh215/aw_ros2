#ifndef CONTROL_MODE_HPP_
#define CONTROL_MODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;

enum NavMode{
  JOY = 1,
  NN = 2
};

class ControlNode: public rclcpp::Node
{
public:

  ControlNode();
  
private:

  void joy_vel_callback(Twist::SharedPtr msg);

  void nn_vel_callback(Twist::SharedPtr msg);

  void joy_callback(Joy::SharedPtr msg);

  void timer_cmd_vel();

  rclcpp::Subscription<Twist>::SharedPtr joy_twist_sub_;
  rclcpp::Subscription<Twist>::SharedPtr nn_twist_sub_;
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr vel_timer_;

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  double selected_speed_, selected_angular_;
  double joy_speed_, joy_angular_;
  double nn_speed_, nn_angular_;

  bool deadman;
  NavMode nav_mode_;
};

#endif // CONTROL_MODE_HPP_