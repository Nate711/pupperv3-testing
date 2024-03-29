#include "motor_controller_generic.hpp"

#include <algorithm>
#include <future>
#include <iostream>
#include <memory>
#include <numeric>
#include <thread>
#include "math.h"

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

namespace pupperv3 {

void print_sent_received_debug(const MotorInterface &interface) {
  std::stringstream ss;
  ss << "Sent: " << interface.send_counts() << " Received: " << interface.receive_counts()
     << " Missing resp: " << interface.missing_reply_counts();
  SPDLOG_INFO("{}", ss.str());
}

template <int N>
MotorController<N>::MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                                    const ActuatorVector &endstop_positions_degs,
                                    const ActuatorVector &calibration_directions,
                                    std::unique_ptr<MotorInterface> motor_interface,
                                    bool print_position_debug, bool print_sent_received)
    : motor_interface_(std::move(motor_interface)),
      position_kp_(position_kp),
      speed_kp_(speed_kp),
      max_speed_(max_speed),
      calibration_directions_(calibration_directions),
      is_robot_calibrated_(false),
      busy_(false),
      print_position_debug_(print_position_debug),
      print_sent_received_(print_sent_received) {
  if (motor_interface_->actuator_config().size() != N) {
    // Exceptions call object destructors except if thrown in constructor
    motor_interface_.reset();
    SPDLOG_ERROR(
        "Number of actuators in motor interface ({}) does not match that for motor controller "
        "({})",
        motor_interface_->actuator_config().size(), N);
    throw std::runtime_error("Number of actuator mismatch");
  }
  measured_endstop_positions_ = ActuatorVector::Zero();
  endstop_positions_ = endstop_positions_degs * M_PI / 180.0F;

  // Set calibrated_motors_ to have size N and value false
  calibrated_motors_ = std::vector<bool>(N, false);
}

template <int N>
MotorController<N>::~MotorController() {
  SPDLOG_INFO("Destroying motor controller...");
}

template <int N>
void MotorController<N>::begin() {
  using namespace std::chrono_literals;
  SPDLOG_INFO("Initializing motor controller.");
  motor_interface_->start_read_threads();
  std::this_thread::sleep_for(10ms);
  motor_interface_->initialize_motors();
  motor_interface_->write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);
  std::this_thread::sleep_for(10ms);
  velocity_control(ActuatorVector::Zero(), true);
}

template <int N>
void MotorController<N>::stop() {
  motor_interface_->command_all_stop();
}

template <int N>
void MotorController<N>::throw_on_not_calibrated(const std::string &msg) {
  if (!is_robot_calibrated_) {
    throw RobotNotCalibratedException();
  }
}

template <int N>
float MotorController<N>::raw_to_calibrated(float value, float measured_endstop_position,
                                            float endstop_position) {
  return value - measured_endstop_position + endstop_position;
}

template <int N>
typename MotorController<N>::ActuatorVector MotorController<N>::raw_to_calibrated(
    const ActuatorVector &value, const ActuatorVector &measured_endstop_position,
    const ActuatorVector &endstop_position) {
  return value - measured_endstop_position + endstop_position;
}

template <int N>
typename MotorController<N>::ActuatorVector MotorController<N>::raw_actuator_positions() {
  ActuatorVector data;
  const auto motor_data = motor_interface_->motor_data_safe();
  for (size_t i = 0; i < N; i++) {
    data(i) = motor_data.at(i).common.output_rads;
  }
  return data;
}

template <int N>
typename MotorController<N>::ActuatorVector MotorController<N>::actuator_positions() {
  return raw_to_calibrated(raw_actuator_positions(), measured_endstop_positions_,
                           endstop_positions_);
}

template <int N>
typename MotorController<N>::ActuatorVector MotorController<N>::actuator_velocities() {
  ActuatorVector data;
  auto motor_data = motor_interface_->motor_data_safe();
  for (size_t i = 0; i < N; i++) {
    data(i) = motor_data.at(i).common.output_rads_per_sec;
  }
  return data;
}

template <int N>
typename MotorController<N>::ActuatorVector MotorController<N>::actuator_efforts() {
  ActuatorVector data;
  auto motor_data = motor_interface_->motor_data_safe();
  for (size_t i = 0; i < N; i++) {
    data(i) = motor_data.at(i).common.current;
  }
  return data;
}

template <int N>
void MotorController<N>::position_control(const ActuatorVector &goal_positions) {
  position_control(goal_positions, position_kp_, max_speed_, false);
}

// TODO: don't use vectors, use arrays or something
template <int N>
void MotorController<N>::position_control(const ActuatorVector &goal_positions, float position_kp,
                                          float max_speed, bool override_busy) {
  if (is_busy() && !override_busy) {
    SPDLOG_INFO("Ignoring position_control call because robot is busy");
    return;
  }
  throw_on_not_calibrated("position_control: robot not calibrated");

  auto latest_data = motor_interface_->motor_data_safe();
  ActuatorVector corrected_actuator_positions = actuator_positions();
  ActuatorVector position_error = goal_positions - corrected_actuator_positions;
  ActuatorVector velocity_command = position_error * position_kp;  // in rotor deg/s

  // Clip velocity commands (unncessary except for printing bc do it in velocity_control)
  velocity_command = velocity_command.cwiseMax(-max_speed).cwiseMin(max_speed);

  velocity_control(velocity_command, override_busy);

  if (print_position_debug_) {
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    SPDLOG_INFO("Pos reference: {}", goal_positions.transpose().format(CleanFmt));
    SPDLOG_INFO("Pos corrected: {}", corrected_actuator_positions.transpose().format(CleanFmt));
    SPDLOG_INFO("Pos raw: {}", raw_actuator_positions().transpose().format(CleanFmt));
  }

  if (print_sent_received_) {
    print_sent_received_debug(*motor_interface_);
  }
}

/// @brief Command velocity of single motor
/// @tparam N
/// @param motor_id
/// @param velocity
template <int N>
void MotorController<N>::velocity_control_single_motor(int motor_index, float velocity,
                                                       bool override_busy) {
  if (is_busy() && !override_busy) {
    SPDLOG_INFO("Ignoring single-motor velocity_control call because robot is busy");
    return;
  }
  double clamped_velocity_command = std::min(std::max(velocity, -max_speed_), max_speed_);
  motor_interface_->command_velocity(motor_interface_->actuator_config(motor_index),
                                     clamped_velocity_command);
  std::this_thread::sleep_for(std::chrono::microseconds(kDelayAfterCommand));
}

/* Command velocity
 *
 * TODO(nathankau): Bug if multiple blocking_move() are called from different threads
 *
 * Args:
 *  vel_targets: rotor deg/s
 *  override_busy: set true to command velocity even when controller thinks robot is busy
 */
template <int N>
void MotorController<N>::velocity_control(const ActuatorVector &velocity_command,
                                          bool override_busy) {
  if (is_busy() && !override_busy) {
    SPDLOG_INFO("Ignoring velocity_control call because robot is busy");
    return;
  }

  ActuatorVector clamped_velocity_command =
      velocity_command.cwiseMax(-max_speed_).cwiseMin(max_speed_);
  for (int i = 0; i < N; i++) {
    motor_interface_->command_velocity(motor_interface_->actuator_config(i),
                                       clamped_velocity_command(i));
    std::this_thread::sleep_for(std::chrono::microseconds(kDelayAfterCommand));
  }
}

template <int N>
bool MotorController<N>::is_calibrated() {
  // Is atomic so no need to lock
  return is_robot_calibrated_;
}

template <int N>
void MotorController<N>::calibrate_motors(const std::atomic_bool &should_stop) {
  calibrate_motors(should_stop, kDefaultCalibrationParams);
}

// A function that prints out with spdlog the elements of a vector using ssstream
template <typename T>
void log_vector(const std::vector<T> &vec) {
  std::stringstream ss;
  ss << "[";
  for (const auto &i : vec) {
    ss << i << " ";
  }
  ss << "]";
  SPDLOG_INFO(ss.str());
}

template <int N>
void MotorController<N>::calibrate_motors(const std::atomic_bool &should_stop,
                                          std::vector<int> motors_to_calibrate,
                                          const CalibrationParams &params) {
  SPDLOG_INFO("--------------- Starting calibration ---------------");
  log_vector(motors_to_calibrate);

  using namespace std::chrono_literals;

  // Spin off async tasks to calibrate motors
  std::vector<std::future<float>> measured_endstop_position_futures;
  for (int i : motors_to_calibrate) {
    measured_endstop_position_futures.emplace_back(
        std::async(&MotorController<N>::calibrate_motor, this, std::ref(should_stop), i, params));
  }

  // Wait for calibrations to complete. Careful about indexing measured enstop positions
  // because they are not in the same order as motors_to_calibrate
  for (size_t i = 0; i < motors_to_calibrate.size(); i++) {
    int motor_index = motors_to_calibrate[i];
    measured_endstop_positions_(motor_index) = measured_endstop_position_futures[i].get();
  }

  // Command motors which were calibrated to zero speed
  for (int i : motors_to_calibrate) {
    velocity_control_single_motor(i, 0, true);
  }

  SPDLOG_INFO("Finished calibration:");
  log_vector(motors_to_calibrate);

  SPDLOG_INFO("Calibration status:");
  log_vector(calibrated_motors_);
}

template <int N>
void MotorController<N>::calibrate_motors(const std::atomic_bool &should_stop,
                                          const CalibrationParams &params) {
  SPDLOG_INFO("--------------- Starting full-robot calibration ---------------");
  is_robot_calibrated_ = false;

  // Construct a vector from 0 to N-1
  std::vector<int> motor_indices(N);
  std::iota(std::begin(motor_indices), std::end(motor_indices), 0);

  // Calibrate all motors simultaneously
  calibrate_motors(should_stop, motor_indices, params);

  is_robot_calibrated_ = true;
  SPDLOG_INFO("Finished full-robot calibration");
}

template <int N>
float MotorController<N>::calibrate_motor(const std::atomic_bool &should_stop, int motor_index,
                                          const CalibrationParams &params) {
  using namespace std::chrono_literals;

  SPDLOG_INFO("Calibrating motor {}", motor_index);
  is_robot_calibrated_ = false;

  {
    std::lock_guard<std::mutex> lock(calibrated_motors_mutex_);
    calibrated_motors_.at(motor_index) = false;
  }

  int loops_at_endstop = 0;
  float command_velocity = params.calibration_speed * calibration_directions_(motor_index);
  float measured_endstop_position = 0.0;
  int averaging_ticks = params.calibration_threshold - params.start_averaging_ticks;

  MotorID motor_id = motor_interface_->actuator_config(motor_index);  // 0-indexed
  motor_interface_->write_pid_ram(motor_id, 0, 0, params.calibration_speed_kp, 0,
                                  MotorInterface::kDefaultIqKp, MotorInterface::kDefaultIqKi);
  std::this_thread::sleep_for(1ms);

  while (true) {
    if (should_stop) {
      SPDLOG_WARN("-- Stopping calibration of motor {} prematurely --", motor_index);
      return 0.0;
    }

    // Command motor
    velocity_control_single_motor(motor_index, command_velocity, true);

    // Get latest data from the motor in a thread-safe way
    auto motor_data = motor_interface_->motor_data_safe().at(motor_index).common;

    // Read data
    float output_rads = motor_data.output_rads;
    float output_rad_per_sec = motor_data.output_rads_per_sec;
    float current = motor_data.current;

    // Check if at endstop
    if ((std::abs(output_rad_per_sec) < params.speed_threshold) &&
        (std::abs(current) > params.current_threshold)) {
      loops_at_endstop += 1;
    }

    // Check if the number of ticks at the endstop has been reached
    if (loops_at_endstop >= params.calibration_threshold) {
      command_velocity = 0.0;
      if (print_position_debug_) {
        SPDLOG_INFO("Motor {} done.", motor_index);
      }

      // Reset PID gains
      motor_interface_->write_pid_ram(motor_id, 0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                      MotorInterface::kDefaultIqKi);

      // Mark motor as calibrated
      // Mark robot as calibrated if all motors calibrated
      {
        std::lock_guard<std::mutex> lock(calibrated_motors_mutex_);
        calibrated_motors_.at(motor_index) = true;
        if (std::all_of(calibrated_motors_.begin(), calibrated_motors_.end(),
                        [](bool v) { return v; })) {
          is_robot_calibrated_ = true;
        }
      }

      // Return the average position at the endstop
      return measured_endstop_position;
    }
    // Start averaging position at endstop after a few ticks at the endstop
    else if (loops_at_endstop >= params.start_averaging_ticks) {
      measured_endstop_position += output_rads / averaging_ticks;
    }

    // Debug printing
    if (print_position_debug_) {
      std::stringstream ss;
      ss << "Idx: " << motor_index << " v: " << output_rad_per_sec << " I: " << current
         << " ticks: " << loops_at_endstop << " v*: " << command_velocity;
      SPDLOG_INFO("{}", ss.str());
    }

    if (print_sent_received_) {
      print_sent_received_debug(*motor_interface_);
    }

    // Sleep for a bit
    std::this_thread::sleep_for(params.sleep_time);
  }
}

template <int N>
void MotorController<N>::blocking_move(const std::atomic_bool &should_stop, float max_speed,
                                       float position_kp, uint8_t move_speed_kp,
                                       const ActuatorVector &goal_position, float speed_tolerance,
                                       int wait_ticks) {
  SPDLOG_INFO("--------------- Starting blocking move-----------");
  set_busy();
  using namespace std::chrono_literals;
  auto sleep_time = 5000us;  // 200hz

  // Set special blocking move gains
  motor_interface_->write_pid_ram_to_all(0, 0, move_speed_kp, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);
  // Need to wait 1ms here to prevent can bus from dying
  std::this_thread::sleep_for(1ms);
  for (int ticks = 0;; ticks++) {
    if (should_stop) {
      SPDLOG_WARN("-- Stopping blocking move prematurely --");
      break;
    }
    position_control(goal_position, position_kp, max_speed, true);
    std::this_thread::sleep_for(sleep_time);

    if ((actuator_velocities().template lpNorm<Eigen::Infinity>() < speed_tolerance) &&
        (ticks > wait_ticks)) {
      break;
    }
    if (print_sent_received_) {
      print_sent_received_debug(*motor_interface_);
    }
  }

  // Reset PD gains to default
  std::this_thread::sleep_for(1ms);
  motor_interface_->write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);
  std::this_thread::sleep_for(1ms);

  // Set motors back to zero velocity
  velocity_control(ActuatorVector::Zero(), true);
  std::this_thread::sleep_for(5ms);
  velocity_control(ActuatorVector::Zero(), true);

  // Unblock robot controller
  set_available();
  SPDLOG_INFO("--------------- Finished blocking move-----------");
}

// template class MotorController<1>;
// template class MotorController<2>;
template class MotorController<3>;
// template class MotorController<4>;
// template class MotorController<5>;
template class MotorController<6>;
template class MotorController<12>;

}  // namespace pupperv3