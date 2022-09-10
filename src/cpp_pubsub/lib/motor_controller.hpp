#pragma once
#include <vector>
#include "motor_interface.hpp"

template <int kServosPerChannel>
class MotorController
{
public:
    MotorController(float position_kp, uint8_t speed_kp, float max_speed, vector<CANChannel> motor_connections);
    void begin();
    void run(vector<array<float, kServosPerChannel>> goal_positions);
    MotorData motor_data_copy(CANChannel bus, uint8_t motor_id);
    std::array<std::array<MotorData, kServosPerChannel>, kNumCANChannels> motor_data_copy();

private:
    MotorInterface<kServosPerChannel> motor_interface_;
    float position_kp_;
    float speed_kp_;
    float max_speed_;
};