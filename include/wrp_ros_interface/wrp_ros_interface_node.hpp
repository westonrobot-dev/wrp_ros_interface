#ifndef WRP_ROS_INTERFACE_NODE_HPP
#define WRP_ROS_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
// #include <tier4_api_utils/tier4_api_utils.hpp>
// #include <vehicle_info_util/vehicle_info_util.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
// #include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/engage.hpp>
// #include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
// #include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
// #include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
// #include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

// #include <tier4_api_msgs/msg/door_status.hpp>
// #include <tier4_external_api_msgs/srv/set_door.hpp>
// #include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
// #include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
// #include <tier4_vehicle_msgs/msg/control_mode.hpp>
// #include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
// #include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
// #include <tier4_vehicle_msgs/srv/control_mode_request.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

namespace westonrobot {
class WrpRosInterfaceNode : public rclcpp::Node {
 public:
  WrpRosInterfaceNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WrpRosInterfaceNode(){};

 private:
  // ----- ROS Node Parameters -----
  std::string base_frame_id_;
  std::string cmd_vel_topic_;
  int command_timeout_ms_;  // vehicle_cmd timeout [ms]
  double loop_rate_;        // [Hz]
  double tire_radius_;      // [m]
  double wheel_base_;       // [m]
  // ----- Internal Variables -----
  // ----- Published Messages -----
  // ----- Subscribers & Publishers & Services -----
  rclcpp::Subscription<
      autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
      control_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
      velocity_report_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  // ----- Timers -----
  // rclcpp::TimerBase::SharedPtr report_timer_;
  // ----- Callbacks -----
  void callbackControlCmd(
      const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr
          msg);
  void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool ReadParameters();
  bool SetupInterfaces();
};  // WrpRosInterfaceNode
}  // namespace westonrobot

#endif /* WRP_ROS_INTERFACE_NODE_HPP */
