#include "motor_controller.hpp"

#include <algorithm>
#include <thread>

#define TEMPLATE_HEADER template <int kServosPerChannel>
#define MOTOR_CONTROLLER MotorController<kServosPerChannel>

TEMPLATE_HEADER
MOTOR_CONTROLLER::MotorController(float position_kp, uint8_t speed_kp, float max_speed,
                                  std::vector<CANChannel> motor_connections)
    : motor_interface_(motor_connections),
      num_can_buses_(motor_connections.size()),
      position_kp_(position_kp),
      speed_kp_(speed_kp),
      max_speed_(max_speed),
      is_robot_calibrated_(false) {}

TEMPLATE_HEADER
MOTOR_CONTROLLER::~MotorController() {}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::begin() {
  std::cout << "Initializing motor controller." << std::endl;
  motor_interface_.initialize_canbuses();
  motor_interface_.initialize_motors();
  motor_interface_.start_read_threads();
  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKi);
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(const MOTOR_CONTROLLER::ActuatorCommand &goal_positions) {
  position_control(goal_positions, max_speed_);
}

// TODO: don't use vectors, use arrays or something
TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(const MOTOR_CONTROLLER::ActuatorCommand &goal_positions,
                                        float max_speed) {
  if (goal_positions.size() != num_can_buses_) {
    std::cout << "WARNING: attempting to use " << goal_positions.size() << " can buses when only "
              << num_can_buses_ << " were initialized\n";
  }
  auto latest_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
      MotorData motor_data = latest_data.at(bus_idx).at(motor_id - 1);
      float goal_position = goal_positions.at(bus_idx).at(motor_id - 1);
      std::cout << "p*: " << goal_position << " p: " << motor_data.common.output_rads << " ";
      float position_error =
          goal_position - motor_data.common.output_rads;  // scale kp down by factor of 10*180/pi
      float velocity_command = position_error * position_kp_;  // in rotor deg/s
      velocity_command = std::clamp(velocity_command, -max_speed, max_speed);
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
void MOTOR_CONTROLLER::velocity_control(const MOTOR_CONTROLLER::ActuatorCommand &vel_targets) {
  if (vel_targets.size() != num_can_buses_) {
    std::cout << "WARNING: attempting to use " << vel_targets.size() << " can buses when only "
              << num_can_buses_ << " were initialized\n";
  }
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
      float velocity_command = vel_targets.at(bus_idx).at(motor_id - 1);
      velocity_command = std::clamp(velocity_command, -max_speed_, max_speed_);
      motor_interface_.command_velocity(bus, motor_id, velocity_command);
    }
  }
}

TEMPLATE_HEADER
typename MOTOR_CONTROLLER::RobotMotorData MOTOR_CONTROLLER::motor_data_safe() {
  return motor_interface_.motor_data_safe();
}

TEMPLATE_HEADER
MotorData MOTOR_CONTROLLER::motor_data_safe(CANChannel bus, uint8_t motor_id) {
  return motor_interface_.motor_data_safe(bus, motor_id);
}

TEMPLATE_HEADER
int MOTOR_CONTROLLER::flat_index(int bus_idx, int motor_idx) {
  return bus_idx * kServosPerChannel + motor_idx;
}

TEMPLATE_HEADER
int MOTOR_CONTROLLER::flat_size() { return kServosPerChannel * num_can_buses_; }

// TODO: store calibration result position somewhere
TEMPLATE_HEADER
void MOTOR_CONTROLLER::calibrate_motors(const std::atomic<bool> &should_stop) {
  std::cout << "--------------- BEGINNING CALIBRATION ---------------" << std::endl;
  using namespace std::chrono_literals;
  const float velocity_target = 1000;  // rotor deg/s
  // TODO: make this cal directions a parameter so that user isn't fixed to 12 motors or this
  // pattern
  const std::vector<int> cal_directions = {-1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, -1};
  const float calibration_speed_kp = 10;
  const float speed_threshold = 0.05;
  const float current_threshold = 1.0;
  const int calibration_threshold = 50;
  const int start_averaging_ticks = 10;
  const int averaging_ticks = calibration_threshold - start_averaging_ticks;
  const std::chrono::duration sleep_time = 2000us;

  is_robot_calibrated_ = false;
  std::vector<int> loops_at_endstop(num_can_buses_ * kServosPerChannel, 0);
  std::vector<float> command_velocities(num_can_buses_ * kServosPerChannel, 0.0);
  std::vector<float> endstop_positions(num_can_buses_ * kServosPerChannel, 0.0);

  for (size_t i = 0; i < command_velocities.size(); i++) {
    command_velocities.at(i) = velocity_target * cal_directions.at(i);
  }

  auto calibrated = [calibration_threshold](int i) { return i >= calibration_threshold; };

  motor_interface_.write_pid_ram_to_all(0, 0, calibration_speed_kp, 0,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKi);

  // ----------------- SUPER DUPER IMPORTANT ------------------------//
  // ----------------- SUPER DUPER IMPORTANT ------------------------//
  // -----------Reset to all_of done testing----  ------------------------//
  // ----------------- SUPER DUPER IMPORTANT ------------------------//
  while (!should_stop &&
         !std::any_of(loops_at_endstop.begin(), loops_at_endstop.end(), calibrated)) {
    RobotMotorData motor_data;

    motor_data = motor_interface_.motor_data_safe();
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
      CANChannel bus = kAllCANChannels.at(bus_idx);
      for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
        int idx = flat_index(bus_idx, motor_id - 1);
        float command_velocity = command_velocities.at(idx);

        motor_interface_.command_velocity(bus, motor_id, command_velocity);

        float output_rads = motor_data.at(bus_idx).at(motor_id - 1).common.output_rads;
        float output_rad_per_sec =
            motor_data.at(bus_idx).at(motor_id - 1).common.output_rads_per_sec;
        float current = motor_data.at(bus_idx).at(motor_id - 1).common.current;
        if ((abs(output_rad_per_sec) < speed_threshold) && (abs(current) > current_threshold)) {
          loops_at_endstop.at(idx) += 1;
        }
        if (calibrated(loops_at_endstop.at(idx))) {
          command_velocities.at(idx) = 0.0;
        } else if (loops_at_endstop.at(idx) >= start_averaging_ticks) {
          endstop_positions.at(idx) += output_rads / averaging_ticks;
        }
        std::cout << "p: " << output_rads << " v*: " << command_velocity
                  << " v: " << output_rad_per_sec << " I: " << current
                  << " ticks: " << loops_at_endstop.at(idx)
                  << " endstop: " << endstop_positions.at(idx);
      }
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(sleep_time);
  }
  is_robot_calibrated_ = true;
  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKi);

  std::cout << "------------ FINISHED CALIBRATION ----------" << std::endl;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::blocking_move(const std::atomic<bool> &should_stop, float max_speed,
                                     float move_speed_kp, const ActuatorCommand &goal_position,
                                     float speed_tolerance, int wait_ticks) {
  std::cout << "--------------- Starting blocking move-----------" << std::endl;
  using namespace std::chrono_literals;
  auto sleep_time = 2000us;
  auto float_within_tolerance = [speed_tolerance](float val) {
    return std::abs(val) < speed_tolerance;
  };

  auto vec_within_tolerance = [float_within_tolerance](std::vector<float> vec) {
    return std::all_of(vec.begin(), vec.end(), float_within_tolerance);
  };

  motor_interface_.write_pid_ram_to_all(0, 0, move_speed_kp, 0,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKi);

  // Need to wait 1ms here to prevent can bus from dying
  std::this_thread::sleep_for(1000us);

  std::vector<float> actuator_vels(flat_size(), 0.0);
  int ticks = 0;
  while (!should_stop && !(vec_within_tolerance(actuator_vels) && (ticks > wait_ticks))) {
    position_control(goal_position, max_speed);

    auto motor_data = motor_interface_.motor_data_safe();
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++) {
      CANChannel bus = kAllCANChannels.at(bus_idx);
      for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
        actuator_vels.at(flat_index(bus_idx, motor_id - 1)) =
            motor_data.at(bus_idx).at(motor_id - 1).common.output_rads_per_sec;
      }
    }
    ticks++;
    std::this_thread::sleep_for(sleep_time);
  }

  motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKp,
                                        MotorInterface<kServosPerChannel>::kDefaultIqKi);
  std::this_thread::sleep_for(1000us);

  std::cout << "--------------- Finished blocking move-----------" << std::endl;
}

template class MotorController<1>;
template class MotorController<2>;
template class MotorController<3>;
template class MotorController<4>;
template class MotorController<5>;
template class MotorController<6>;
