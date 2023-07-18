#pragma once
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "motor_interface_generic.hpp"

using namespace std::chrono_literals;

namespace pupperv3 {

template <class T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii) {
    os << " " << *ii;
  }
  os << "]";
  return os;
}

struct RobotNotCalibratedException : public std::exception {
  const char *what() const throw() { return "Robot not calibrated"; }
};

template <int N>
class MotorController {
 public:
  using ActuatorVector = Eigen::Matrix<float, N, 1>;
  using ActuatorVectorI = Eigen::Matrix<int, N, 1>;

  MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                  const ActuatorVector &endstop_positions_degs,
                  const ActuatorVector &calibration_directions,
                  std::unique_ptr<MotorInterface> motor_interface,
                  bool print_position_debug = false, bool print_sent_receive = false);
  ~MotorController();
  void begin();
  void stop();

  void position_control(const ActuatorVector &goal_positions);
  void position_control(const ActuatorVector &goal_positions, float position_kp, float max_speed,
                        bool override_busy);

  void velocity_control_single_motor(int motor_index, float velocity, bool override_busy);

  // WARNING: UNTESTED
  void velocity_control(const ActuatorVector &vel_targets, bool override_busy = false);

  // MotorData motor_data_safe(CANChannel bus, uint8_t motor_id);

  // RobotMotorData motor_data_safe();

  /*
   * Returns outputs' angle in rads
   */
  ActuatorVector actuator_positions();

  /*
   * Returns output's velocity in rads/s
   */
  ActuatorVector actuator_velocities();

  /*
   * Returns: Current in A
   */
  ActuatorVector actuator_efforts();

  bool is_calibrated();

  struct CalibrationParams {
    float calibration_speed;
    float calibration_speed_kp;
    float speed_threshold;
    float current_threshold;
    int calibration_threshold;
    int start_averaging_ticks;
    std::chrono::microseconds sleep_time;
  };

  /// @brief Calibrates the motors specified in motor_to_calibrate.
  /// @param should_stop A reference to an atomic boolean variable that indicates whether the
  /// calibration process should be stopped prematurely.
  /// @param motor_to_calibrate A vector of motor indices to calibrate.
  /// @param params Calibration params (see CalibrationParams)
  void calibrate_motors(const std::atomic_bool &should_stop, std::vector<int> motor_to_calibrate,
                        const CalibrationParams &params);

  /// @brief Calibrates all motors one by one.
  /// @param should_stop A reference to an atomic boolean variable that indicates whether the
  /// calibration process should be stopped prematurely.
  /// @param params Calibration params (see CalibrationParams)
  void calibrate_motors(const std::atomic_bool &should_stop, const CalibrationParams &params);

  void calibrate_motors(const std::atomic_bool &should_stop);

  /// @brief Calibrates a single motor.
  /// @param should_stop A reference to an atomic boolean variable that indicates whether the
  /// calibration process should be stopped prematurely.
  /// @param motor_index The 0-based index of the motor to calibrate.
  /// @param params Calibration params (see CalibrationParams)
  /// @return The calibrated motor's position in degrees.
  float calibrate_motor(const std::atomic_bool &should_stop, int motor_index,
                        const CalibrationParams &params);

  inline bool is_busy() { return busy_; }

  void blocking_move(const std::atomic<bool> &should_stop, float max_speed, float position_kp,
                     uint8_t speed_kp, const ActuatorVector &goal_position,
                     float speed_tolerance = 0.01, int wait_ticks = 20);

  inline ActuatorVectorI micros_since_last_read() const {
    return ActuatorVectorI(motor_interface_->micros_since_last_read().data());
  }

  inline ActuatorVectorI receive_counts() const {
    return ActuatorVectorI(motor_interface_->receive_counts().data());
  }

  inline ActuatorVectorI send_counts() const {
    return ActuatorVectorI(motor_interface_->send_counts().data());
  }

  static constexpr CalibrationParams kDefaultCalibrationParams = {
      .calibration_speed = 750,  // rotor deg/s
      .calibration_speed_kp = 30,
      .speed_threshold = 0.05,
      .current_threshold = 4.0,
      .calibration_threshold = 20,
      .start_averaging_ticks = 10,
      .sleep_time = 5000us  // 200hz calibration loop
  };

 private:
  static float raw_to_calibrated(float val, float measured_endstop_position,
                                 float endstop_position);
  static ActuatorVector raw_to_calibrated(const ActuatorVector &val,
                                          const ActuatorVector &measured_endstop_position,
                                          const ActuatorVector &endstop_position);

  void throw_on_not_calibrated(const std::string &msg);
  inline void set_busy() { busy_ = true; }
  inline void set_available() { busy_ = false; }
  /*
   * Returns outputs' angle in rads
   */
  ActuatorVector raw_actuator_positions();

  static constexpr int kDelayAfterCommand = 200;

  std::mutex motor_interface_lock_;
  std::unique_ptr<MotorInterface> motor_interface_;
  size_t num_can_buses_;
  float position_kp_;
  float speed_kp_;
  float max_speed_;
  ActuatorVector calibration_directions_;
  std::atomic<bool> is_robot_calibrated_;
  std::mutex calibrated_motors_mutex_;
  std::vector<bool> calibrated_motors_;  // 0-indexed indices of calibrated motors
  ActuatorVector measured_endstop_positions_;
  ActuatorVector endstop_positions_;

  // Blocking moves set busy to true to prevent other control actions from taking place
  std::atomic<bool> busy_;

  bool print_position_debug_;
  bool print_sent_received_;
};
}  // namespace pupperv3