#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using String = std_msgs::msg::String;
using Float32 = std_msgs::msg::Float32;
using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using AckermannCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using LongitudinalCommand = autoware_auto_control_msgs::msg::LongitudinalCommand;
using LateralCommand = autoware_auto_control_msgs::msg::AckermannLateralCommand;
using namespace std::placeholders;

const float ACCELERATION = 0.25f;

class SimpleControlConverter: public rclcpp::Node
{
public:
    SimpleControlConverter(): Node("simple_controller_node")
    {
        
        qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        this->command_ = GearCommand::DRIVE;
        
        this->time_scale_ = 1.0;
        this->step_ = 0.0;
        this->auto_ = false;
        this->cmd_vel_sub_ = this->create_subscription<Twist>("cmd_vel",
            10, std::bind(&SimpleControlConverter::twist_callback, this, _1));

        this->cmd_vel_nn_sub_ = this->create_subscription<Twist>("nn_cmd_vel",
            10, std::bind(&SimpleControlConverter::twist_nn_callback, this, _1));

        this->joy_input_ = this->create_subscription<Joy>("joy",
            10, std::bind(&SimpleControlConverter::joy_callback, this, _1));
            
        this->ackermann_cmd_pub_ = this->create_publisher<AckermannCommand>("/control/command/control_cmd", this->qos_);
        this->gear_cmd_pub_ = this->create_publisher<GearCommand>("/control/command/gear_cmd", this->qos_);
        this->time_scale_pub_ = this->create_publisher<Float32>("/time_scale", this->qos_);
        this->mode_pub_ = this->create_publisher<String>("steering_change", this->qos_);

        this->timer_ = this->create_wall_timer(std::chrono::seconds(1), 
            std::bind(&SimpleControlConverter::change_time_scale, this));
        this->switch_mode_timer_ = this->create_wall_timer(std::chrono::seconds(1),
            std::bind(&SimpleControlConverter::switch_mode, this));

        this->switch_mode_timer_->cancel();
        this->timer_->cancel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Simple Control node started. Control the vehicle using /cmd_vel topic.");
    }

private:

    void switch_mode(){
        String msg;
        if (this->mode_){
            msg.data = "FALSE";
            this->mode_pub_->publish(msg);
            this->mode_ = false;
        } else {
            msg.data = "TRUE";
            this->mode_ = true;
        }
        this->mode_pub_->publish(msg);
        this->switch_mode_timer_->cancel();
    }

    void change_time_scale(){
        Float32 msg;
        this->time_scale_ += this->step_;
        if (this->time_scale_ < 0.5){
            this->time_scale_ = 0.5;
        } else if (this->time_scale_ > 1.2){
            this->time_scale_ = 1.2;
        }
        msg.data = this->time_scale_;
        this->time_scale_pub_->publish(msg);
        this->step_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "Time scaled: %.2f" , this->time_scale_);
        this->timer_->cancel();
    }

    void joy_callback(const Joy::SharedPtr msg){

        GearCommand gear_command;
        gear_command.stamp = rclcpp::Clock().now();

        if (msg->buttons[1] == 1){
            this->auto_ = false;
            this->command_ = GearCommand::DRIVE;
        }
        else if (msg->buttons[2] == 1){
            this->auto_ = true;
            this->command_ = GearCommand::DRIVE_2;
        } else if (abs(msg->axes[7]) == 1.0 && this->step_ == 0.0){
            this->step_ = msg->axes[7] * 0.1;
            this->timer_->reset();
        } else if (msg->buttons[4] == 1){
            switch_mode_timer_->reset();
        }

        gear_command.command = this->command_;
        this->gear_cmd_pub_->publish(gear_command);
    }

    void twist_nn_callback(const Twist::SharedPtr msg){
        LongitudinalCommand longitudinal;
        longitudinal.speed = msg->linear.x;
        longitudinal.acceleration = ACCELERATION;

        LateralCommand lateral;
        lateral.steering_tire_angle = msg->angular.z;

        AckermannCommand send_command;
        send_command.stamp = rclcpp::Clock().now();
        send_command.longitudinal = longitudinal;
        send_command.lateral = lateral;

        if (this->auto_) {
            ackermann_cmd_pub_->publish(send_command);
        }
    }

    void twist_callback(const Twist::SharedPtr msg){
        LongitudinalCommand longitudinal;
        longitudinal.speed = msg->linear.x;
        longitudinal.acceleration = ACCELERATION;

        LateralCommand lateral;
        lateral.steering_tire_angle = msg->angular.z;

        AckermannCommand send_command;
        send_command.stamp = rclcpp::Clock().now();
        send_command.longitudinal = longitudinal;
        send_command.lateral = lateral;

        if (!this->auto_){
            ackermann_cmd_pub_->publish(send_command);
        }
    }

    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_nn_sub_;
    rclcpp::Subscription<Joy>::SharedPtr joy_input_;

    rclcpp::Publisher<AckermannCommand>::SharedPtr ackermann_cmd_pub_;
    rclcpp::Publisher<GearCommand>::SharedPtr gear_cmd_pub_;
    rclcpp::Publisher<Float32>::SharedPtr time_scale_pub_;
    rclcpp::Publisher<String>::SharedPtr mode_pub_;

    rclcpp::QoS qos_ = rclcpp::QoS(10);
    rclcpp::TimerBase::SharedPtr timer_, switch_mode_timer_;
    
    int command_;
    double time_scale_, step_;
    bool auto_, mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleControlConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}