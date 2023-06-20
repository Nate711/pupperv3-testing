#pragma once

#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "motor_controller_generic.hpp"
#include "motor_interface_generic.hpp"

namespace pupperv3 {

class WatchdogTriggered : public std::exception {
 public:
  WatchdogTriggered(const std::string& msg) : msg_(msg) {}
  virtual const char* what() const noexcept override { return msg_.c_str(); }

 private:
  std::string msg_;
};

template <int K_SERVOS>
class MotorControllerNode : public rclcpp::Node {
 public:
  using ActuatorVector = typename MotorController<K_SERVOS>::ActuatorVector;
  /// @brief
  /// @param rate Rate at which to publish joint state messages and to send CAN commands to motors
  /// @param motor_controller Unique ptr to motor controller object
  /// @param joint_names Joint names
  /// @param default_position
  /// @param verbose
  /// @param very_verbose
  MotorControllerNode(float rate, std::unique_ptr<MotorController<K_SERVOS>> motor_controller,
                      const std::array<std::string, K_SERVOS>& joint_names,
                      const ActuatorVector& default_position, bool verbose = false,
                      bool very_verbose = false);
  ~MotorControllerNode();
  void startup();

 private:
  void startup_thread_fn();

  void publish_callback();

  // @brief Callback for CAN bus watchdog
  // TODO(nathankau): The timer for this callback is set to start at node creation which
  // so even if you send a command that does not require a response, the watchdog will trigger after
  // the specified timeout.
  // A potential solution is to set a boolean per motor indicating if a
  // response is expected and reset it after reading, although this approach may ignore requests
  // that require a response if they are followed by requests that do not require a response.
  void watchdog_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscriber_;
  sensor_msgs::msg::JointState joint_state_message_;
  void joint_command_callback(pupper_interfaces::msg::JointCommand joint_command);

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  static constexpr int kWatchDogWarningUS = 10000;  // watchdog should be run twice as frequent
  static constexpr int kWatchDogTimeoutUS = 50000;
  float publish_rate_;
  std::unique_ptr<MotorController<K_SERVOS>> motor_controller_;
  ActuatorVector default_position_;
  pupper_interfaces::msg::JointCommand latest_joint_command_;
  std::thread startup_thread_;
  std::atomic<bool> stop_;

  bool verbose_;
  bool very_verbose_;
};
}  // namespace pupperv3