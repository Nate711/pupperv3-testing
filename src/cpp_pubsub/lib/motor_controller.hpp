#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "motor_interface.hpp"

struct RobotNotCalibratedException : public std::exception {
  const char *what() const throw() { return "Robot not calibrated"; }
};

template <int ServosPerChannel>
class MotorController {
 public:
  using RobotMotorData = std::array<std::array<MotorData, ServosPerChannel>, kNumCANChannels>;

  template <typename T>
  using ActuatorMatrix = std::vector<std::array<T, ServosPerChannel>>;

  MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                  std::vector<CANChannel> motor_connections);
  ~MotorController();
  void begin();

  void position_control(const ActuatorMatrix<float> &goal_positions);
  void position_control(const ActuatorMatrix<float> &goal_positions, float max_speed);

  // WARNING: UNTESTED
  void velocity_control(const ActuatorMatrix<float> &vel_targets);

  // MotorData motor_data_safe(CANChannel bus, uint8_t motor_id);

  // RobotMotorData motor_data_safe();

  /*
   * Returns outputs' angle in rads
   */
  ActuatorMatrix<float> actuator_positions();

  /*
   * Returns output's velocity in rads/s
   */
  ActuatorMatrix<float> actuator_velocities();

  /*
   * Returns: Current in A
   */
  ActuatorMatrix<float> actuator_efforts();

  bool is_calibrated();

  void calibrate_motors(const std::atomic<bool> &should_stop);

  void blocking_move(const std::atomic<bool> &should_stop, float max_speed, float speed_kp,
                     const ActuatorMatrix<float> &goal_position, float speed_tolerance = 0.01,
                     int wait_ticks = 20);

 private:
  template <typename T>
  static void zero_actuator_matrix(ActuatorMatrix<T> &data, int n) {
    data.clear();
    for (int i = 0; i < n; i++) {
      data.push_back(std::array<T, ServosPerChannel>{});
    }
  }
  static float raw_to_calibrated(float val, float measured_endstop_position,
                                 float endstop_position);
  void throw_on_not_calibrated(const std::string &msg);
  template <typename T>
  void warn_actuator_matrix_invalid(const ActuatorMatrix<T> &mat);
  int flat_index(int bus_idx, int motor_idx);
  int flat_size();

  std::mutex motor_interface_lock_;
  MotorInterface<ServosPerChannel> motor_interface_;
  size_t num_can_buses_;
  float position_kp_;
  float speed_kp_;
  float max_speed_;
  std::atomic<bool> is_robot_calibrated_;
  ActuatorMatrix<float> measured_endstop_positions_;
  ActuatorMatrix<float> endstop_positions_;
};