#include "motor_controller.hpp"

#include <algorithm>
#include <thread>

#define TEMPLATE_HEADER template <int kServosPerChannel>
#define MOTOR_CONTROLLER MotorController<kServosPerChannel>

TEMPLATE_HEADER
MOTOR_CONTROLLER::MotorController(float position_kp, uint8_t speed_kp,
                                  float max_speed,
                                  std::vector<CANChannel> motor_connections)
    : motor_interface_(motor_connections), num_can_buses_(motor_connections.size()),
      position_kp_(position_kp), speed_kp_(speed_kp),
      max_speed_(max_speed),
      is_robot_calibrated_(false), stop_calibration_(false)
{
}

TEMPLATE_HEADER
MOTOR_CONTROLLER::~MotorController()
{
  std::cout << "Waiting for calibration thread to end if started" << std::endl;
  stop_calibration_ = true;
  if (calibration_thread_)
  {
    calibration_thread_->join();
  }
  std::cout << "Joined calibration_thread_. Done destroying MotorController" << std::endl;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::begin()
{
  std::cout << "Initializing motor controller." << std::endl;
  std::unique_lock<std::mutex> lock(motor_interface_lock_);
  motor_interface_.initialize_canbuses();
  motor_interface_.initialize_motors();
  motor_interface_.start_read_threads();
  motor_interface_.write_pid_ram_to_all(
      0, 0, speed_kp_, 0, MotorInterface<kServosPerChannel>::kDefaultIqKp,
      MotorInterface<kServosPerChannel>::kDefaultIqKi);
}

// TODO: don't use vectors, use arrays or something
TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(
    std::vector<std::array<float, kServosPerChannel>> goal_positions)
{
  std::unique_lock<std::mutex> lock(motor_interface_lock_);
  if (goal_positions.size() != num_can_buses_)
  {
    std::cout << "WARNING: attempting to use " << goal_positions.size()
              << " can buses when only " << num_can_buses_
              << " were initialized\n";
  }
  auto latest_data = motor_interface_.motor_data_safe();
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++)
  {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++)
    {
      MotorData motor_data = latest_data.at(bus_idx).at(motor_id - 1);
      float goal_position = goal_positions.at(bus_idx).at(motor_id - 1);
      std::cout << "gpos: " << goal_position
                << " pos: " << motor_data.common.output_rads << " ";
      float position_error =
          goal_position -
          motor_data.common.output_rads;                      // scale kp down by factor of 10*180/pi
      float velocity_command = position_error * position_kp_; // in rotor deg/s
      velocity_command = std::clamp(velocity_command, -max_speed_, max_speed_);
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
    std::vector<std::array<float, kServosPerChannel>> vel_targets)
{
  std::unique_lock<std::mutex> lock(motor_interface_lock_);
  if (vel_targets.size() != num_can_buses_)
  {
    std::cout << "WARNING: attempting to use " << vel_targets.size()
              << " can buses when only " << num_can_buses_
              << " were initialized\n";
  }
  for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++)
  {
    CANChannel bus = kAllCANChannels.at(bus_idx);
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++)
    {
      float velocity_command = vel_targets.at(bus_idx).at(motor_id - 1);
      velocity_command = std::clamp(velocity_command, -max_speed_, max_speed_);
      motor_interface_.command_velocity(bus, motor_id, velocity_command);
    }
  }
}

TEMPLATE_HEADER
typename MotorController<kServosPerChannel>::RobotMotorData
MOTOR_CONTROLLER::motor_data_safe()
{
  std::unique_lock<std::mutex> lock(motor_interface_lock_);
  return motor_interface_.motor_data_safe();
}

TEMPLATE_HEADER
MotorData MOTOR_CONTROLLER::motor_data_safe(CANChannel bus, uint8_t motor_id)
{
  std::unique_lock<std::mutex> lock(motor_interface_lock_);
  return motor_interface_.motor_data_safe(bus, motor_id);
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::start_calibration()
{
  calibration_thread_ = std::make_shared<std::thread>(&MOTOR_CONTROLLER::simulation_thread, this);
}

TEMPLATE_HEADER
int MOTOR_CONTROLLER::flat_index(int bus_idx, int motor_idx)
{
  return bus_idx * kServosPerChannel + motor_idx;
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::simulation_thread()
{
  std::cout << "--------------- BEGINNING CALIBRATION ---------------" << std::endl;
  using namespace std::chrono_literals;
  const float velocity_target = 1000; // rotor deg/s
  const std::vector<int> cal_directions = {-1, 1, 1,
                                           1, -1, -1,
                                           -1, 1, 1,
                                           1, -1, -1};
  const float calibration_speed_kp = 10;
  const float speed_threshold = 0.05;
  const float current_threshold = 1.0;
  const int calibration_threshold = 50;
  const std::chrono::duration sleep_time = 2000us;

  is_robot_calibrated_ = false;
  std::vector<int> loops_at_endstop(num_can_buses_ * kServosPerChannel, 0);
  std::vector<float> command_velocities(num_can_buses_ * kServosPerChannel, 0.0);
  for (size_t i = 0; i < command_velocities.size(); i++)
  {
    command_velocities.at(i) = velocity_target * cal_directions.at(i);
  }

  auto calibrated = [calibration_threshold](int i)
  { return i >= calibration_threshold; };

  {
    std::unique_lock<std::mutex> lock(motor_interface_lock_);
    motor_interface_.write_pid_ram_to_all(
        0, 0, calibration_speed_kp, 0,
        MotorInterface<kServosPerChannel>::kDefaultIqKp,
        MotorInterface<kServosPerChannel>::kDefaultIqKi);
  }

  while (!stop_calibration_ &&
         !std::all_of(loops_at_endstop.begin(),
                      loops_at_endstop.end(),
                      calibrated))
  {

    RobotMotorData motor_data;
    {
      std::unique_lock<std::mutex> lock(motor_interface_lock_);
      motor_data = motor_interface_.motor_data_safe();
    }
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++)
    {
      CANChannel bus = kAllCANChannels.at(bus_idx);
      for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++)
      {
        int idx = flat_index(bus_idx, motor_id - 1);
        float command_velocity = command_velocities.at(idx);
        {
          std::unique_lock<std::mutex> lock(motor_interface_lock_);
          motor_interface_.command_velocity(bus, motor_id, command_velocity);
        }
        float output_rad_per_sec = motor_data.at(bus_idx).at(motor_id - 1).common.output_rads_per_sec;
        float current = motor_data.at(bus_idx).at(motor_id - 1).common.current;
        if (abs(output_rad_per_sec) < speed_threshold && abs(current > current_threshold))
        {
          loops_at_endstop.at(idx) += 1;
        }
        if (calibrated(loops_at_endstop.at(idx)))
        {
          command_velocities.at(idx) = 0.0;
        }
        std::cout << "v*: " << command_velocity << " v: " << output_rad_per_sec << " I: " << current << " ticks: " << loops_at_endstop.at(flat_index(bus_idx, motor_id - 1)) << " ";
      }
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(sleep_time);
  }
  is_robot_calibrated_ = true;
  {
    std::unique_lock<std::mutex> lock(motor_interface_lock_);
    motor_interface_.write_pid_ram_to_all(
        0, 0, speed_kp_, 0,
        MotorInterface<kServosPerChannel>::kDefaultIqKp,
        MotorInterface<kServosPerChannel>::kDefaultIqKi);
  }
  std::cout << "------------ FINISHED CALIBRATION ----------" << std::endl;
}

template class MotorController<1>;
template class MotorController<2>;
template class MotorController<3>;
template class MotorController<4>;
template class MotorController<5>;
template class MotorController<6>;
