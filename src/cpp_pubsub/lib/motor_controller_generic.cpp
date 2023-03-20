#include "motor_controller_generic.hpp"

#include <algorithm>
#include <thread>
#include "math.h"

namespace pupperv3 {

template <int N>
MotorController<N>::MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                                    const ActuatorVector &endstop_positions_degs,
                                    const ActuatorVector &calibration_directions,
                                    std::unique_ptr<MotorInterface> motor_interface)
    : motor_interface_(std::move(motor_interface)),
      position_kp_(position_kp),
      speed_kp_(speed_kp),
      max_speed_(max_speed),
      calibration_directions_(calibration_directions),
      is_robot_calibrated_(false),
      busy_(false) {
  if (motor_interface_->actuator_config().size() != N) {
    // Exceptions call object destructors except if thrown in constructor
    motor_interface_.reset();
    throw std::runtime_error(
        "Number of actuators in motor interface does not match N for motor controller");
  }
  measured_endstop_positions_ = ActuatorVector::Zero();
  endstop_positions_ = endstop_positions_degs * M_PI / 180.0F;
}

template <int N>
MotorController<N>::~MotorController() {
  std::cout << "Motor controller destructor called" << std::endl;
}

template <int N>
void MotorController<N>::begin() {
  std::cout << "Initializing motor controller." << std::endl;
  // motor_interface_->initialize_canbuses();
  motor_interface_->initialize_motors();
  motor_interface_->start_read_threads();
  motor_interface_->write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);

  velocity_control(ActuatorVector::Zero());
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
    std::cout << "Ignoring position_control call because robot is busy" << std::endl;
    return;
  }
  throw_on_not_calibrated("position_control: robot not calibrated");

  auto latest_data = motor_interface_->motor_data_safe();
  ActuatorVector corrected_actuator_positions = actuator_positions();
  ActuatorVector position_error = goal_positions - corrected_actuator_positions;
  ActuatorVector velocity_command = position_error * position_kp;  // in rotor deg/s

  // Clip velocity commands (unncessary except for printing bc do it in velocity_control)
  velocity_command = velocity_command.cwiseMax(-max_speed).cwiseMin(max_speed);

  std::cout << "p*: " << goal_positions << " cp: " << corrected_actuator_positions
            << " rp: " << raw_actuator_positions() << "\n";

  velocity_control(velocity_command);
}

/* Command velocity
 *
 * Args:
 *  vel_targets: rotor deg/s
 */
template <int N>
void MotorController<N>::velocity_control(const ActuatorVector &velocity_command) {
  if (is_busy()) {
    std::cout << "Ignoring velocity_control call because robot is busy" << std::endl;
    return;
  }

  // sometimes need to call this function while not calibrated eg calibration and before
  // calibration? throw_on_not_calibrated("velocity_control: robot not calibrated");

  ActuatorVector clamped_velocity_command =
      velocity_command.cwiseMax(-max_speed_).cwiseMin(max_speed_);
  for (size_t i = 0; i < N; i++) {
    motor_interface_->command_velocity(motor_interface_->actuator_config().at(i),
                                       clamped_velocity_command(i));
  }
}

template <int N>
bool MotorController<N>::is_calibrated() {
  return is_robot_calibrated_;
}

// TODO: store calibration result position somewhere
template <int N>
void MotorController<N>::calibrate_motors(const std::atomic<bool> &should_stop) {
  std::cout << "--------------- BEGINNING CALIBRATION ---------------" << std::endl;
  using namespace std::chrono_literals;
  const float velocity_target = 750;  // rotor deg/s
  const float calibration_speed_kp = 30;
  const float speed_threshold = 0.05;
  const float current_threshold = 4.0;
  const int calibration_threshold = 20;
  const int start_averaging_ticks = 10;
  const int averaging_ticks = calibration_threshold - start_averaging_ticks;
  const std::chrono::duration sleep_time = 5000us;  // 2000us;

  is_robot_calibrated_ = false;

  ActuatorVectorI loops_at_endstop = ActuatorVectorI::Zero();
  ActuatorVector command_velocities = ActuatorVector::Zero();
  measured_endstop_positions_ = ActuatorVector::Zero();
  command_velocities = velocity_target * calibration_directions_;

  motor_interface_->write_pid_ram_to_all(
      0, 0, calibration_speed_kp, 0, MotorInterface::kDefaultIqKp, MotorInterface::kDefaultIqKi);

  // DEBUG ONLY, OVERRIDING CALIBRATION
  // loops_at_endstop.at(0) = 100;
  // loops_at_endstop.at(1) = 100;
  // loops_at_endstop.at(2) = 100;
  // loops_at_endstop.at(3) = 100;
  // loops_at_endstop.at(4) = 100;
  // loops_at_endstop.at(5) = 100;
  // loops_at_endstop.at(4 + 6) = 100;

  while (!should_stop && !(loops_at_endstop.minCoeff() >= calibration_threshold)) {
    // Command motors
    velocity_control(command_velocities);

    // Get latest data from motors in thread-safe way
    auto motor_data = motor_interface_->motor_data_safe();

    // Iterate over each motor
    for (size_t idx = 0; idx < N; idx++) {
      // Read data
      float output_rads = motor_data.at(idx).common.output_rads;
      float output_rad_per_sec = motor_data.at(idx).common.output_rads_per_sec;
      float current = motor_data.at(idx).common.current;

      // Check if at endstop
      if ((std::abs(output_rad_per_sec) < speed_threshold) &&
          (std::abs(current) > current_threshold)) {
        loops_at_endstop(idx) += 1;
      }

      // Check if number of ticks at the endstop has been reached
      if (loops_at_endstop(idx) >= calibration_threshold) {
        command_velocities(idx) = 0.0;
      }  // Start average position at endstop after a few ticks at the endstop
      else if (loops_at_endstop(idx) >= start_averaging_ticks) {
        measured_endstop_positions_(idx) += output_rads / averaging_ticks;
      }

      // Debug printing
      std::cout << " || v: " << output_rad_per_sec << " I: " << current
                << " ticks: " << loops_at_endstop(idx) << " v*: " << command_velocities(idx);
    }

    std::cout << std::endl;
    std::this_thread::sleep_for(sleep_time);
  }
  is_robot_calibrated_ = true;
  motor_interface_->write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);

  std::cout << "------------ FINISHED CALIBRATION ----------" << std::endl;
}
template <int N>
void MotorController<N>::blocking_move(const std::atomic<bool> &should_stop, float max_speed,
                                       float position_kp, uint8_t move_speed_kp,
                                       const ActuatorVector &goal_position, float speed_tolerance,
                                       int wait_ticks) {
  std::cout << "--------------- Starting blocking move-----------" << std::endl;
  set_busy();
  using namespace std::chrono_literals;
  auto sleep_time = 5000us;

  // Set special blocking move gains
  motor_interface_->write_pid_ram_to_all(0, 0, move_speed_kp, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);
  // Need to wait 1ms here to prevent can bus from dying
  std::this_thread::sleep_for(1000us);

  for (int ticks = 0;; ticks++) {
    position_control(goal_position, position_kp, max_speed, true);
    std::this_thread::sleep_for(sleep_time);
    if (should_stop) {
      std::cout << "----------Blocking move cancelled-----------" << std::endl;
      break;
    }
    if ((actuator_velocities().template lpNorm<Eigen::Infinity>() < speed_tolerance) &&
        (ticks > wait_ticks)) {
      break;
    }
  }

  // Reset PD gains to default
  std::this_thread::sleep_for(1000us);
  motor_interface_->write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface::kDefaultIqKp,
                                         MotorInterface::kDefaultIqKi);
  std::this_thread::sleep_for(1000us);

  // Set motors back to zero velocity
  velocity_control(ActuatorVector::Zero());

  // Unblock robot controller
  set_available();
  std::cout << "--------------- Finished blocking move-----------" << std::endl;
}

// template class MotorController<1>;
// template class MotorController<2>;
template class MotorController<3>;
// template class MotorController<4>;
// template class MotorController<5>;
template class MotorController<6>;
template class MotorController<12>;

}  // namespace pupperv3