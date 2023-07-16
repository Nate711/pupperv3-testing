#pragma once
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "motor_interface_generic.hpp"

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

  void velocity_control_single_motor(int motor_index, double velocity);

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

  /**
   * @brief Calibrates all motors one by one.
   *
   * This function calibrates all the motors one by one. Each motor is calibrated
   * independently until it reaches the endstop position. The calibration process
   * sets the desired velocity, monitors the motor's position and current, and stops
   * when the motor reaches the endstop condition.
   *
   * @param should_stop A reference to an atomic boolean variable that indicates
   *                    whether the calibration process should be stopped prematurely.
   * @return void
   */
  void calibrate_motors(const std::atomic_bool &should_stop);

  /**
   * @brief Calibrates a specific motor.
   *
   * This function calibrates a specific motor identified by its index. The calibration
   * process sets the desired velocity for the motor, monitors its position and current,
   * and stops when the motor reaches the endstop condition.
   *
   * @param should_stop A reference to an atomic boolean variable that indicates
   *                    whether the calibration process should be stopped prematurely.
   * @param motor_index The index of the motor to be calibrated (0-based index).
   * @param calibration_velocity The velocity (sign and magnitude) that the motor will rotate at in
   * rotor degrees per second.
   * @param calibration_speed_kp The proportional gain for the calibration speed PID.
   * @param speed_threshold The threshold for the motor speed to detect endstop condition.
   * @param current_threshold The threshold for the motor current to detect endstop condition.
   * @param calibration_threshold The number of ticks needed to detect the endstop condition.
   * @param start_averaging_ticks The number of ticks before starting to average the endstop
   * position.
   * @param averaging_ticks The number of ticks used to average the endstop position.
   * @param sleep_time The sleep time between calibration iterations (calibration loop frequency).
   * @return void
   */
  void calibrate_motor(const std::atomic_bool &should_stop, int motor_index,
                       float calibration_velocity, float calibration_speed_kp,
                       float speed_threshold, float current_threshold, int calibration_threshold,
                       int start_averaging_ticks, int averaging_ticks,
                       std::chrono::duration sleep_time);

  inline bool is_busy() { return busy_; }

  void blocking_move(const std::atomic<bool> &should_stop, float max_speed, float position_kp,
                     uint8_t speed_kp, const ActuatorVector &goal_position,
                     float speed_tolerance = 0.01, int wait_ticks = 20);

  inline ActuatorVectorI micros_since_last_read() const {
    return ActuatorVectorI(motor_interface_->micros_since_last_read().data());
  }

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
  ActuatorVector measured_endstop_positions_;
  ActuatorVector endstop_positions_;

  // Blocking moves set busy to true to prevent other control actions from taking place
  std::atomic<bool> busy_;

  bool print_position_debug_;
  bool print_sent_received_;
};
}  // namespace pupperv3