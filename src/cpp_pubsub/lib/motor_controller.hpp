#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include "motor_interface.hpp"

template <int ServosPerChannel>
class MotorController {
 public:
  using RobotMotorData = std::array<std::array<MotorData, ServosPerChannel>, kNumCANChannels>;
  using ActuatorCommand = std::vector<std::array<float, ServosPerChannel>>;

  MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                  std::vector<CANChannel> motor_connections);
  ~MotorController();
  void begin();

  void position_control(const ActuatorCommand &goal_positions);
  void position_control(const ActuatorCommand &goal_positions, float max_speed);

  void velocity_control(const ActuatorCommand &vel_targets);

  MotorData motor_data_safe(CANChannel bus, uint8_t motor_id);

  RobotMotorData motor_data_safe();

  void calibrate_motors(const std::atomic<bool> &should_stop);

  void blocking_move(const std::atomic<bool> &should_stop, float max_speed, float speed_kp,
                     const ActuatorCommand &goal_position, float speed_tolerance, int wait_ticks);

 private:
  int flat_index(int bus_idx, int motor_idx);
  int flat_size();

  std::mutex motor_interface_lock_;
  MotorInterface<ServosPerChannel> motor_interface_;
  size_t num_can_buses_;
  float position_kp_;
  float speed_kp_;
  float max_speed_;
  std::atomic<bool> is_robot_calibrated_;
};