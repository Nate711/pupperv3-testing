#pragma once
#include "motor_interface.hpp"
#include <array>
#include <mutex>
#include <vector>
#include <memory>

template <int kServosPerChannel>
class MotorController
{
public:
  using RobotMotorData = std::array<std::array<MotorData, kServosPerChannel>, kNumCANChannels>;

  MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                  std::vector<CANChannel> motor_connections);
  
  ~MotorController();

  void begin();

  void position_control(
      std::vector<std::array<float, kServosPerChannel>> goal_positions);
  void velocity_control(
      std::vector<std::array<float, kServosPerChannel>> vel_targets);

  MotorData motor_data_safe(CANChannel bus, uint8_t motor_id);

  std::array<std::array<MotorData, kServosPerChannel>, kNumCANChannels>
  motor_data_safe();

  void start_calibration();

private:
  int flat_index(int bus_idx, int motor_idx);
  void simulation_thread();

  std::mutex motor_interface_lock_;
  MotorInterface<kServosPerChannel> motor_interface_;
  size_t num_can_buses_;
  float position_kp_;
  float speed_kp_;
  float max_speed_;
  bool is_robot_calibrated_;

  std::shared_ptr<std::thread> calibration_thread_;
  std::atomic<bool> stop_calibration_;
};