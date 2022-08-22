#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <array>
#include <unordered_map>
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

const int kServosPerChannel = 3;
const int kNumCANChannels = 4;

enum class CANChannel
{
    CAN0 = 0,
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

const vector<CANChannel> kAllCANChannels = {CANChannel::CAN0, CANChannel::CAN1, CANChannel::CAN2, CANChannel::CAN3};

struct MotorData
{
    uint8_t error = 0;
    uint8_t motor_id = 0;
    float multi_angle = 0;
};

class MotorInterface
{
public:
    MotorInterface(unordered_map<CANChannel, vector<uint32_t>> motor_connections, int bitrate);
    ~MotorInterface();
    void initialize_canbuses();
    void close_canbuses();
    void initialize_motors();
    void request_multi_angle(CANChannel bus, uint32_t motor_id);
    MotorData read_multi_angle(CANChannel bus);
    void start_read_threads();
    array<array<MotorData, kServosPerChannel>, kNumCANChannels> latest_data();

private:
    void initialize_bus(CANChannel bus);
    void initialize_motor(CANChannel bus, uint32_t motor_id);
    uint32_t can_id(uint32_t motor_id);
    uint32_t motor_id(uint32_t can_id);
    void read_thread(CANChannel channel);
    void send(CANChannel bus, uint32_t motor_id, const array<uint8_t, 8> &payload);

    unordered_map<CANChannel, string> kChannelLookup = {{CANChannel::CAN0, "can0"}, {CANChannel::CAN1, "can1"}};
    array<int, 4> canbus_to_fd_;
    unordered_map<CANChannel, vector<uint32_t>> motor_connections_;
    const int bitrate_;
    bool initialized_;
    atomic<bool> should_read_;

    const uint8_t kStartup0 = 0x76;
    const uint8_t kStartup1 = 0x88;
    const uint8_t kStartup2 = 0x77;
    const uint8_t kGetMultiAngle = 0x92;
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
