#include "motor_controller.hpp"

#include <algorithm>
#include <thread>
#include "math.h"

#define TEMPLATE_HEADER template <int ServosPerChannel>
#define MOTOR_CONTROLLER MotorController<ServosPerChannel>

TEMPLATE_HEADER
MOTOR_CONTROLLER::MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                                  std::vector<CANChannel> motor_connections)
    : motor_interface_(motor_connections),
      num_can_buses_(motor_connections.size()),
      position_kp_(position_kp),
      speed_kp_(speed_kp),
      max_speed_(max_speed),
      is_robot_calibrated_(false),
      busy_(false) {
  zero_actuator_matrix(measured_endstop_positions_, num_can_buses_);
  zero_actuator_matrix(endstop_positions_, num_can_buses_);

  // TODO: replace with parameter in the future
  endstop_positions_ = {{-135, 90, 68, 135, -90, -68}, {-135, 90, 68, 135, -90, -68}};
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    for (int motor_idx = 0; motor_idx < ServosPerChannel; motor_idx++) {
      endstop_positions_.at(bus_idx).at(motor_idx) *= M_PI / 180.0F;
    }
  }
}

TEMPLATE_HEADER
MOTOR_CONTROLLER::~MotorController() {
  std::cout << "Motor controller destructor called" << std::endl;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::begin() {
  std::cout << "Initializing motor controller." << std::endl;
  motor_interface_.initialize_canbuses();
  motor_interface_.initialize_motors();
  motor_interface_.start_read_threads();
  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKi);
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    for (int motor_idx = 0; motor_idx < ServosPerChannel; motor_idx++) {
      CANChannel bus = kAllCANChannels.at(bus_idx);
      motor_interface_.command_velocity(bus, motor_idx + 1, 0.0);
    }
  }
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::stop() { motor_interface_.command_all_stop(); }

TEMPLATE_HEADER
void MOTOR_CONTROLLER::throw_on_not_calibrated(const std::string &msg) {
  if (!is_robot_calibrated_) {
    throw RobotNotCalibratedException();
  }
}

TEMPLATE_HEADER
template <typename T>
void MOTOR_CONTROLLER::warn_actuator_matrix_invalid(const ActuatorMatrix<T> &mat) {
  if (mat.size() != num_can_buses_) {
    std::cout << "WARNING: attempting to use " << mat.size() << " can buses when only "
              << num_can_buses_ << " were initialized\n";
  }
}

TEMPLATE_HEADER
float MOTOR_CONTROLLER::raw_to_calibrated(float value, float measured_endstop_position,
                                          float endstop_position) {
  return value - measured_endstop_position + endstop_position;
}

TEMPLATE_HEADER
typename MOTOR_CONTROLLER::ActuatorMatrix<float> MOTOR_CONTROLLER::actuator_positions() {
  ActuatorMatrix<float> data(num_can_buses_);
  auto motor_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    for (int motor_idx = 0; motor_idx < ServosPerChannel; motor_idx++) {
      auto raw = motor_data.at(bus_idx).at(motor_idx).common.output_rads;
      auto measured_endstop = measured_endstop_positions_.at(bus_idx).at(motor_idx);
      auto endstop = endstop_positions_.at(bus_idx).at(motor_idx);
      data.at(bus_idx).at(motor_idx) = raw_to_calibrated(raw, measured_endstop, endstop);
    }
  }
  return data;
}

TEMPLATE_HEADER
typename MOTOR_CONTROLLER::ActuatorMatrix<float> MOTOR_CONTROLLER::actuator_velocities() {
  ActuatorMatrix<float> data(num_can_buses_);
  auto motor_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    for (int motor_idx = 0; motor_idx < ServosPerChannel; motor_idx++) {
      data.at(bus_idx).at(motor_idx) =
          motor_data.at(bus_idx).at(motor_idx).common.output_rads_per_sec;
    }
  }
  return data;
}

TEMPLATE_HEADER
typename MOTOR_CONTROLLER::ActuatorMatrix<float> MOTOR_CONTROLLER::actuator_efforts() {
  ActuatorMatrix<float> data(num_can_buses_);
  auto motor_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    for (int motor_idx = 0; motor_idx < ServosPerChannel; motor_idx++) {
      data.at(bus_idx).at(motor_idx) = motor_data.at(bus_idx).at(motor_idx).common.current;
    }
  }
  return data;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(
    const MOTOR_CONTROLLER::ActuatorMatrix<float> &goal_positions) {
  position_control(goal_positions, position_kp_, max_speed_, false);
}

// TODO: don't use vectors, use arrays or something
TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(
    const MOTOR_CONTROLLER::ActuatorMatrix<float> &goal_positions, float position_kp,
    float max_speed, bool override_busy) {
  if (is_busy() && !override_busy) {
    std::cout << "Ignoring position_control call because robot is busy" << std::endl;
    return;
  }
  throw_on_not_calibrated("position_control: robot not calibrated");
  warn_actuator_matrix_invalid<float>(goal_positions);

  auto latest_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= ServosPerChannel; motor_id++) {
      MotorData motor_data = latest_data.at(bus_idx).at(motor_id - 1);
      float goal_position = goal_positions.at(bus_idx).at(motor_id - 1);
      float corrected_position = raw_to_calibrated(
          motor_data.common.output_rads, measured_endstop_positions_.at(bus_idx).at(motor_id - 1),
          endstop_positions_.at(bus_idx).at(motor_id - 1));
      float position_error = goal_position - corrected_position;
      float velocity_command = position_error * position_kp;  // in rotor deg/s
      velocity_command = std::clamp(velocity_command, -max_speed, max_speed);
      std::cout << "|| p*: " << goal_position << " cp: " << corrected_position
                << " rp: " << motor_data.common.output_rads << " ";
      motor_interface_.command_velocity(bus, motor_id, velocity_command);
    }
    std::cout << std::endl;
  }
}

/* Command velocity
 *
 * Args:
 *  vel_targets: rotor deg/s
 */
TEMPLATE_HEADER
void MOTOR_CONTROLLER::velocity_control(
    const MOTOR_CONTROLLER::ActuatorMatrix<float> &vel_targets) {
  if (is_busy()) {
    std::cout << "Ignoring velocity_control call because robot is busy" << std::endl;
    return;
  }
  throw_on_not_calibrated("velocity_control: robot not calibrated");
  warn_actuator_matrix_invalid<float>(vel_targets);

  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= ServosPerChannel; motor_id++) {
      float velocity_command = vel_targets.at(bus_idx).at(motor_id - 1);
      velocity_command = std::clamp(velocity_command, -max_speed_, max_speed_);
      motor_interface_.command_velocity(bus, motor_id, velocity_command);
    }
  }
}

// TEMPLATE_HEADER
// typename MOTOR_CONTROLLER::RobotMotorData MOTOR_CONTROLLER::motor_data_safe() {
//   return motor_interface_.motor_data_safe();
// }

// TEMPLATE_HEADER
// MotorData MOTOR_CONTROLLER::motor_data_safe(CANChannel bus, uint8_t motor_id) {
//   return motor_interface_.motor_data_safe(bus, motor_id);
// }

TEMPLATE_HEADER
int MOTOR_CONTROLLER::flat_index(int bus_idx, int motor_idx) {
  return bus_idx * ServosPerChannel + motor_idx;
}

TEMPLATE_HEADER
int MOTOR_CONTROLLER::flat_size() { return ServosPerChannel * num_can_buses_; }

TEMPLATE_HEADER
bool MOTOR_CONTROLLER::is_calibrated() { return is_robot_calibrated_; }

// TODO: store calibration result position somewhere
TEMPLATE_HEADER
void MOTOR_CONTROLLER::calibrate_motors(const std::atomic<bool> &should_stop) {
  std::cout << "--------------- BEGINNING CALIBRATION ---------------" << std::endl;
  using namespace std::chrono_literals;
  const float velocity_target = 750;  // rotor deg/s
  // TODO: make this cal directions a parameter so that user isn't fixed to 12 motors or this
  // pattern
  const std::vector<int> cal_directions = {-1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, -1};
  const float calibration_speed_kp = 30;
  const float speed_threshold = 0.05;
  const float current_threshold = 4.0;
  const int calibration_threshold = 20;
  const int start_averaging_ticks = 10;
  const int averaging_ticks = calibration_threshold - start_averaging_ticks;
  const std::chrono::duration sleep_time = 5000us;  // 2000us;

  is_robot_calibrated_ = false;
  std::vector<int> loops_at_endstop(num_can_buses_ * ServosPerChannel, 0);
  std::vector<float> command_velocities(num_can_buses_ * ServosPerChannel, 0.0);
  zero_actuator_matrix(measured_endstop_positions_, num_can_buses_);

  for (size_t i = 0; i < command_velocities.size(); i++) {
    command_velocities.at(i) = velocity_target * cal_directions.at(i);
  }

  auto calibrated = [calibration_threshold](int val) { return val >= calibration_threshold; };

  motor_interface_.write_pid_ram_to_all(0, 0, calibration_speed_kp, 0,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKi);

  // DEBUG ONLY, OVERRIDING CALIBRATION
  // loops_at_endstop.at(0) = 100;
  // loops_at_endstop.at(1) = 100;
  // loops_at_endstop.at(2) = 100;
  // loops_at_endstop.at(3) = 100;
  // loops_at_endstop.at(4) = 100;
  // loops_at_endstop.at(5) = 100;
  // loops_at_endstop.at(4 + 6) = 100;

  while (!should_stop &&
         !std::all_of(loops_at_endstop.begin(), loops_at_endstop.end(), calibrated)) {
    // Get latest data from motors in thread-safe way
    RobotMotorData motor_data = motor_interface_.motor_data_safe();

    // Iterate over each motor
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
      CANChannel bus = kAllCANChannels.at(bus_idx);
      for (int motor_id = 1; motor_id <= ServosPerChannel; motor_id++) {
        int idx = flat_index(bus_idx, motor_id - 1);

        // Send velocity command
        float command_velocity = command_velocities.at(idx);
        motor_interface_.command_velocity(bus, motor_id, command_velocity);

        // Read data
        float output_rads = motor_data.at(bus_idx).at(motor_id - 1).common.output_rads;
        float output_rad_per_sec =
            motor_data.at(bus_idx).at(motor_id - 1).common.output_rads_per_sec;
        float current = motor_data.at(bus_idx).at(motor_id - 1).common.current;

        // Check if at endstop
        if ((std::abs(output_rad_per_sec) < speed_threshold) &&
            (std::abs(current) > current_threshold)) {
          loops_at_endstop.at(idx) += 1;
        }

        // Check if number of ticks at the endstop has been reached
        if (calibrated(loops_at_endstop.at(idx))) {
          command_velocities.at(idx) = 0.0;
        }  // Start average position at endstop after a few ticks at the endstop
        else if (loops_at_endstop.at(idx) >= start_averaging_ticks) {
          measured_endstop_positions_.at(bus_idx).at(motor_id - 1) += output_rads / averaging_ticks;
        }

        // Debug printing
        std::cout << " || v: " << output_rad_per_sec << " I: " << current
                  << " ticks: " << loops_at_endstop.at(idx);
      }
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(sleep_time);
  }
  is_robot_calibrated_ = true;
  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKi);

  std::cout << "------------ FINISHED CALIBRATION ----------" << std::endl;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::blocking_move(const std::atomic<bool> &should_stop, float max_speed,
                                     float position_kp, uint8_t move_speed_kp,
                                     const ActuatorMatrix<float> &goal_position,
                                     float speed_tolerance, int wait_ticks) {
  std::cout << "--------------- Starting blocking move-----------" << std::endl;
  set_busy();
  using namespace std::chrono_literals;
  auto sleep_time = 5000us;
  auto float_within_tolerance = [speed_tolerance](float val) {
    return std::abs(val) < speed_tolerance;
  };
  auto vec_within_tolerance = [float_within_tolerance](std::vector<float> vec) {
    return std::all_of(vec.begin(), vec.end(), float_within_tolerance);
  };

  // Set special blocking move gains
  motor_interface_.write_pid_ram_to_all(0, 0, move_speed_kp, 0,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKi);

  // Need to wait 1ms here to prevent can bus from dying
  std::this_thread::sleep_for(1000us);

  std::vector<float> actuator_vels(flat_size(), 0.0);
  int ticks = 0;
  while (!should_stop && !(vec_within_tolerance(actuator_vels) && (ticks > wait_ticks))) {
    position_control(goal_position, position_kp, max_speed, true);

    auto motor_data = motor_interface_.motor_data_safe();
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
      for (int motor_id = 1; motor_id <= ServosPerChannel; motor_id++) {
        actuator_vels.at(flat_index(bus_idx, motor_id - 1)) =
            motor_data.at(bus_idx).at(motor_id - 1).common.output_rads_per_sec;
      }
    }
    ticks++;
    std::this_thread::sleep_for(sleep_time);
  }
  if (should_stop) {
    std::cout << "----------Blocking move cancelled-----------" << std::endl;
  }

  // Reset PD gains to default
  std::this_thread::sleep_for(1000us);
  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<ServosPerChannel>::kDefaultIqKi);
  std::this_thread::sleep_for(1000us);

  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= ServosPerChannel; motor_id++) {
      motor_interface_.command_velocity(bus, motor_id, 0);
    }
  }

  set_available();
  std::cout << "--------------- Finished blocking move-----------" << std::endl;
}

// template class MotorController<1>;
// template class MotorController<2>;
// template class MotorController<3>;
// template class MotorController<4>;
// template class MotorController<5>;
template class MotorController<6>;
