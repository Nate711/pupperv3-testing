#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>
#include "motor_controller_generic_node.hpp"

DEFINE_double(rate, 200.0, "Rate at which to command motors and publish joint state");
DEFINE_bool(verbose, false, "True to print debug info like micros since last receive");
DEFINE_bool(very_verbose, false,
            "True to print debug info like position control info and sent received info");

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
constexpr int K_SERVOS = 6;

int main(int argc, char* argv[]) {
  spdlog::set_pattern("[%H:%M:%S.%e] [%^%7l%$] [%45s:%#] %^%v%$");
  rclcpp::init(argc, argv);

  // Parse the command-line arguments using gflags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Print loop rate
  SPDLOG_INFO("Joint state publish_rate: {}", FLAGS_rate);

  // Create config
  pupperv3::MotorID id1{pupperv3::CANChannel::CAN0, 1};
  pupperv3::MotorID id2{pupperv3::CANChannel::CAN0, 2};
  pupperv3::MotorID id3{pupperv3::CANChannel::CAN0, 3};
  pupperv3::MotorID id4{pupperv3::CANChannel::CAN1, 4};
  pupperv3::MotorID id5{pupperv3::CANChannel::CAN1, 5};
  pupperv3::MotorID id6{pupperv3::CANChannel::CAN1, 6};
  pupperv3::ActuatorConfiguration actuator_config{{id1, id2, id3, id4, id5, id6}};

  auto interface = std::make_unique<pupperv3::MotorInterface>(actuator_config, false);

  // 20000 was on the brink of being too stiff
  // 10000 was on the soft side and robot would fall backwards often
  float position_kp = 15000.0;  // 15000 is good default. units rotor deg/s per output rad
  uint8_t speed_kp = 5;         // 5 is good default. units A/rotor deg/s
  float max_speed = 5000;       // rotor deg/s
  using ActuatorVector = pupperv3::MotorController<K_SERVOS>::ActuatorVector;
  ActuatorVector endstop_positions_degs = {-135, 90, 68, -135, 90, 68};
  ActuatorVector calibration_directions = {-1, 1, 1, -1, 1, 1};
  std::array<std::string, K_SERVOS> joint_names = {"leg_front_r_1", "leg_front_r_2",
                                                   "leg_front_r_3", "leg_front_l_1",
                                                   "leg_front_l_2", "leg_front_l_3"};

  ActuatorVector default_position = {0, 0, 1.0, 0, 0, 1.0};

  auto controller = std::make_unique<pupperv3::MotorController<K_SERVOS>>(
      position_kp, speed_kp, max_speed, endstop_positions_degs, calibration_directions,
      std::move(interface), FLAGS_verbose, FLAGS_very_verbose);
  auto node = std::make_shared<pupperv3::MotorControllerNode<K_SERVOS>>(
      FLAGS_rate, std::move(controller), joint_names, default_position, FLAGS_verbose,
      FLAGS_very_verbose);

  node->startup();
  // TODO: don't start watchdog until position control is going!
  try {
    rclcpp::spin(node);
  } catch (const pupperv3::WatchdogTriggered& e) {
    SPDLOG_ERROR("Caught watchdog exception");
    rclcpp::shutdown();
  }
  SPDLOG_INFO("Rclcpp done spinning");
  rclcpp::shutdown();
  SPDLOG_INFO("Rclcpp shutdown");
  return 0;
}
