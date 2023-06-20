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

  void calibrate_motors(const std::atomic<bool> &should_stop);

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