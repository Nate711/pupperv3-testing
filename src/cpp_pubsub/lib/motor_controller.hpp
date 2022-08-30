#pragma once
#include <vector>
#include "motor_interface.hpp"

template <int kServosPerChannel>
class MotorController
{
public:
    MotorController(float kp, float max_speed, vector<CANChannel> motor_connections, int bitrate);
    void begin();
    void run(vector<array<float, kServosPerChannel>> goal_positions);
    MotorData motor_data_copy(CANChannel bus, uint8_t motor_id);

private:
    MotorInterface<kServosPerChannel> motor_interface_;
    float kp_;
    float max_speed_;
};