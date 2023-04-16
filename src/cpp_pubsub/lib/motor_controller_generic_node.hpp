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
  MotorControllerNode(float rate, std::unique_ptr<MotorController<K_SERVOS>> motor_controller,
                      const std::array<std::string, K_SERVOS>& joint_names,
                      const ActuatorVector& default_position);
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
  static constexpr int kWatchDogTimeoutUS = 10000;  // watchdog should be run twice this frequent

  // TODO: make parameter
  //   const std::string joint_names_[12] = {
  //       "leg_front_r_1", "leg_front_r_2", "leg_front_r_3", "leg_front_l_1",
  //       "leg_front_l_2", "leg_front_l_3", "leg_back_r_1",  "leg_back_r_2",
  //       "leg_back_r_3",  "leg_back_l_1",  "leg_back_l_2",  "leg_back_l_3",
  //   };
  // const std::string joint_names_[K_SERVOS] = {"leg_front_r_1", "leg_front_r_2",
  // "leg_front_r_3",
  //                                             "leg_front_l_1", "leg_front_l_2",
  //                                             "leg_front_l_3"};
  // const std::string joint_names_[K_SERVOS] = {"leg_front_r_1", "leg_front_r_2",
  // "leg_front_r_3"};
  float publish_rate_;
  std::unique_ptr<MotorController<K_SERVOS>> motor_controller_;
  ActuatorVector default_position_;
  pupper_interfaces::msg::JointCommand latest_joint_command_;
  std::thread startup_thread_;
  std::atomic<bool> stop_;
};
}  // namespace pupperv3