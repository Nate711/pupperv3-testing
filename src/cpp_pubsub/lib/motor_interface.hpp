#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <array>
#include <signal.h>
#include <memory>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std;

// const int kServosPerChannel = 6;
const int kNumCANChannels = 4;

const float kCurrentWriteMax = 32.0;
const int kCurrentMultiplier = 2000;
const int kCurrentRawReadMax = 2048;
const float kCurrentReadMax = 33.0;
const int kVelocityMultiplier = 100;
const int kEncoderCountsPerRot = 65536;

enum class CANChannel
{
    CAN0 = 0,
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

const vector<CANChannel> kAllCANChannels = {CANChannel::CAN0, CANChannel::CAN1, CANChannel::CAN2, CANChannel::CAN3};

struct CommonResponse
{
    int16_t encoder_counts = 0;          // counts (-2^15-1 to 2^15)
    int16_t previous_encoder_counts = 0; // previous counts
    float velocity = 0.0;                // degs per sec
    float current = 0.0;                 // Amps
    uint8_t temp = 0;                    // C

    int32_t rotations = 0;        // motor rotations (post-processed)
    float multi_loop_angle = 0.0; // degs (post-processed)
};

struct MultiLoopAngleResponse
{
    float multi_loop_angle = 0.0; // degs
};

struct MotorData
{
    uint8_t error = 0;
    uint8_t motor_id = 0;
    MultiLoopAngleResponse multi_loop;
    CommonResponse common;
};

template <int kServosPerChannel>
class MotorInterface
{
public:
    MotorInterface(vector<CANChannel> motor_connections, int bitrate);
    ~MotorInterface();
    void initialize_canbuses();
    void close_canbuses();
    void initialize_motors();
    void request_multi_angle(CANChannel bus, uint8_t motor_id);
    void command_current(CANChannel bus, uint8_t motor_id, float current);
    void command_velocity(CANChannel bus, uint8_t motor_id, float velocity);
    void command_stop(CANChannel bus, uint8_t motor_id);
    void command_all_stop();
    void read_blocking(CANChannel bus);
    void start_read_threads();
    array<array<MotorData, kServosPerChannel>, kNumCANChannels> latest_data();
    MotorData motor_data_copy(CANChannel bus, uint8_t motor_id);

private:
    void initialize_bus(CANChannel bus);
    void initialize_motor(CANChannel bus, uint8_t motor_id);
    struct can_frame read_canframe_blocking(CANChannel bus);
    uint32_t can_id(uint8_t motor_id);
    uint8_t motor_id(uint32_t can_id);
    void read_thread(CANChannel channel);
    void send(CANChannel bus, uint8_t motor_id, const array<uint8_t, 8> &payload);
    MotorData &motor_data(CANChannel bus, uint8_t motor_id);
    void parse_frame(CANChannel bus, const struct can_frame &frame);
    void multi_angle_update(CANChannel bus, uint8_t motor_id, const struct can_frame &frame);
    void torque_velocity_update(CANChannel bus, uint8_t motor_id, const struct can_frame &frame);
    void update_rotation(CommonResponse &common);
    string channel_str(CANChannel channel)
    {
        switch (channel)
        {
        case CANChannel::CAN0:
            return "can0";
        case CANChannel::CAN1:
            return "can1";
        case CANChannel::CAN2:
            return "can2";
        case CANChannel::CAN3:
            return "can3";
        default:
            cerr << "Invalid can channel" << endl;
            return "";
        }
    }
    array<int, 4> canbus_to_fd_;
    vector<CANChannel> motor_connections_;
    const int bitrate_;
    bool initialized_;
    atomic<bool> should_read_;

    const uint8_t kStartup0 = 0x76;
    const uint8_t kStartup1 = 0x88;
    const uint8_t kStartup2 = 0x77;
    const uint8_t kGetMultiAngle = 0x92;
    const uint8_t kCommandCurrent = 0xA1;
    const uint8_t kCommandVelocity = 0xA2;
    const uint8_t kCommandStop = 0x81;
    const float kDegsPerTick = 0.01;
    const float kSpeedReduction = 0.1;

    const int kTimeoutSeconds = 1;

    // float latest_multi_angle_;
    array<array<MotorData, kServosPerChannel>, kNumCANChannels> latest_data_;
    array<thread, kNumCANChannels> read_threads_;
    mutex latest_data_lock_;

    // DEBUG ONLY
    chrono::system_clock::time_point debug_start_;
};
