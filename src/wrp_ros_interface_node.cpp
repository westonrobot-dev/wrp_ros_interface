/**
 * wrp_ros_interface_node.cpp
 *
 * Created on Wed Sep 07 2022 02:41:03
 *
 * Description:
 *
 * Copyright (c) 2022 Weston Robot Pte. Ltd.
 */
#include "wrp_ros_interface/wrp_ros_interface_node.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

namespace westonrobot {
WrpRosInterfaceNode::WrpRosInterfaceNode(const rclcpp::NodeOptions& options)
    : Node("wrp_ros_interface_node", options) {
  if (!WrpRosInterfaceNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!WrpRosInterfaceNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup ros interfaces");
    rclcpp::shutdown();
  }

}

bool WrpRosInterfaceNode::ReadParameters() {
  // Declare default parameters
  declare_parameter<std::string>("base_frame_id", "base_link");
  declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  declare_parameter<int>("command_timeout_ms", 1000);
  declare_parameter<double>("loop_rate", 30.0);
  declare_parameter<double>("tire_radius", 0.16459);
  declare_parameter<double>("wheel_base", 0.498);

  // Get parameters
  RCLCPP_INFO_STREAM(get_logger(), "--- Parameters loaded are ---");

  get_parameter("base_frame_id", base_frame_id_);
  RCLCPP_INFO_STREAM(get_logger(), "base_frame_id: " << base_frame_id_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "cmd_vel_topic: " << cmd_vel_topic_);

  get_parameter("command_timeout_ms", command_timeout_ms_);
  RCLCPP_INFO_STREAM(get_logger(),
                     "command_timeout_ms: " << command_timeout_ms_);

  get_parameter("loop_rate", loop_rate_);
  RCLCPP_INFO_STREAM(get_logger(), "loop_rate: " << loop_rate_);
  get_parameter("tire_radius", tire_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "tire_radius: " << tire_radius_);
  get_parameter("wheel_base", wheel_base_);
  RCLCPP_INFO_STREAM(get_logger(), "wheel_base: " << wheel_base_);

  RCLCPP_INFO_STREAM(get_logger(), "-----------------------------");

  return true;
}

bool WrpRosInterfaceNode::SetupInterfaces() {
  using std::placeholders::_1;
  using std::placeholders::_2;

  // From autoware
  control_cmd_sub_ = create_subscription<
      autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1,
      std::bind(&WrpRosInterfaceNode::callbackControlCmd, this, _1));

  // To wrp_ros2
  cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);

  return true;
}

void WrpRosInterfaceNode::callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr
        msg) {
  geometry_msgs::msg::Twist diff_cmd;

  diff_cmd.linear.x = msg->longitudinal.speed;

  diff_cmd.angular.z = msg->longitudinal.speed *
                       tan(msg->lateral.steering_tire_angle) / wheel_base_;

  RCLCPP_INFO_STREAM(get_logger(), "Initial Speed: " << msg->longitudinal.speed << " Final Linear X: " << diff_cmd.linear.x);
  RCLCPP_INFO_STREAM(get_logger(), "Initial angle: " << msg->lateral.steering_tire_angle << " Final Angular Z: " << diff_cmd.angular.z);

  cmd_vel_publisher_->publish(diff_cmd);
}

}  // namespace westonrobot

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::WrpRosInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}