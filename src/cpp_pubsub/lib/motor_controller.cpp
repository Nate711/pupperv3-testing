#include "motor_controller.hpp"

#include <algorithm>

#define TEMPLATE_HEADER template <int kServosPerChannel>
#define MOTOR_CONTROLLER MotorController<kServosPerChannel>

TEMPLATE_HEADER
MOTOR_CONTROLLER::MotorController(float position_kp,
                                  uint8_t speed_kp,
                                  float max_speed,
                                  vector<CANChannel> motor_connections) : motor_interface_(motor_connections),
                                                                          position_kp_(position_kp),
                                                                          speed_kp_(speed_kp),
                                                                          max_speed_(max_speed)
{
    num_can_buses_ = motor_connections.size();
}

TEMPLATE_HEADER
void MOTOR_CONTROLLER::begin()
{
    std::cout << "Initializing motor controller." << std::endl;
    motor_interface_.initialize_canbuses();
    motor_interface_.initialize_motors();
    motor_interface_.start_read_threads();
    motor_interface_.write_pid_ram_to_all(0, 0, speed_kp_, 0, MotorInterface<kServosPerChannel>::kDefaultIqKp, MotorInterface<kServosPerChannel>::kDefaultIqKi);
}

// TODO: don't use vectors, use arrays or something
TEMPLATE_HEADER
void MOTOR_CONTROLLER::position_control(vector<array<float, kServosPerChannel>> goal_positions)
{
    if (goal_positions.size() != num_can_buses_)
    {
        std::cout << "WARNING: attempting to use " << goal_positions.size() << " can buses when only " << num_can_buses_ << " were initialized\n";
    }
    auto latest_data = motor_interface_.latest_data();
    for (size_t bus_idx = 0; bus_idx < num_can_buses_; bus_idx++)
    {
        CANChannel bus = kAllCANChannels.at(bus_idx);
        for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++)
        {
            MotorData motor_data = latest_data.at(bus_idx).at(motor_id - 1);
            float goal_position = goal_positions.at(bus_idx).at(motor_id - 1);
            std::cout << "gpos: " << goal_position << " pos: " << motor_data.common.output_rads << " ";
            float position_error = goal_position - motor_data.common.output_rads; // scale kp down by factor of 10*180/pi
            float velocity_command = position_error * position_kp_;                  // in rotor deg/s
            velocity_command = std::clamp(velocity_command, -max_speed_, max_speed_);
            motor_interface_.command_velocity(bus, motor_id, velocity_command);
        }
        std::cout << std::endl;
    }
}

TEMPLATE_HEADER
std::array<std::array<MotorData, kServosPerChannel>, kNumCANChannels> MOTOR_CONTROLLER::motor_data_copy()
{
    return motor_interface_.latest_data();
}

TEMPLATE_HEADER
MotorData MOTOR_CONTROLLER::motor_data_copy(CANChannel bus, uint8_t motor_id)
{
    return motor_interface_.motor_data_copy(bus, motor_id);
}

template class MotorController<1>;
template class MotorController<2>;
template class MotorController<3>;
template class MotorController<4>;
template class MotorController<5>;
template class MotorController<6>;
