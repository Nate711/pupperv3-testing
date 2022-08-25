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

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "motor_interface.hpp"
#include "prof_utils.hpp"

using namespace std;

#define TEMPLATE_HEADER template <int kServosPerChannel>
#define MOTOR_INTERFACE MotorInterface<kServosPerChannel>

TEMPLATE_HEADER
MOTOR_INTERFACE::MotorInterface(vector<CANChannel> motor_connections, int bitrate) : motor_connections_(motor_connections),
                                                                                     bitrate_(bitrate),
                                                                                     initialized_(false),
                                                                                     should_read_(true)
{
    // DEBUG ONLY
    debug_start_ = time_now();
    canbus_to_fd_.fill(-1);
}

TEMPLATE_HEADER
MOTOR_INTERFACE::~MotorInterface()
{
    cout << "Calling motor interface destructor." << endl;
    should_read_.store(false);
    // Causes lots of can socket exceptions / errors
    // for (thread &active_thread : read_threads_)
    // {
    //     cout << "Joining thread. ";
    //     active_thread.join();
    // }
    cout << "Closing can buses" << endl;
    close_canbuses();
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_canbuses()
{
    for (const auto &bus : motor_connections_)
    {
        initialize_bus(bus);
    }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::close_canbuses()
{
    for (const auto &bus : motor_connections_)
    {
        close(canbus_to_fd_.at(static_cast<int>(bus)));
    }
    cout << "Closed can bus sockets." << endl;
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_motors()
{
    for (const auto &bus : motor_connections_)
    {
        for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++)
        {
            cout << "Initializing motor id: " << motor_id << " on channel: " << static_cast<int>(bus) << endl;
            initialize_motor(bus, motor_id);
        }
    }
    initialized_ = true;
}

TEMPLATE_HEADER
array<array<MotorData, kServosPerChannel>, kNumCANChannels> MOTOR_INTERFACE::latest_data()
{
    {
        unique_lock<mutex> lock(latest_data_lock_);
        return latest_data_;
    }
}

/*
Amps
*/
TEMPLATE_HEADER
void MOTOR_INTERFACE::command_current(CANChannel bus, uint32_t motor_id, float current)
{
    int16_t discrete_current = current / kCurrentWriteMax * kCurrentMultiplier;
    uint8_t LSB = discrete_current & 0xFF;
    uint8_t GSB = (discrete_current >> 8) & 0xFF;
    send(bus, motor_id, {kCommandCurrent, 0, 0, 0, LSB, GSB, 0, 0});
}

/*
DEG/S
*/
TEMPLATE_HEADER
void MOTOR_INTERFACE::command_velocity(CANChannel bus, uint32_t motor_id, float velocity)
{
    int32_t discrete_velocity = velocity * kVelocityMultiplier;
    uint8_t LSB = discrete_velocity & 0xFF;
    uint8_t B1 = (discrete_velocity >> 8) & 0xFF;
    uint8_t B2 = (discrete_velocity >> 16) & 0xFF;
    uint8_t B3 = (discrete_velocity >> 24) & 0xFF;
    send(bus, motor_id, {kCommandVelocity, 0, 0, 0, LSB, B1, B2, B3});
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::request_multi_angle(CANChannel bus, uint32_t motor_id)
{
    auto start = time_now();
    send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
    auto stop = time_now();
    // cout << "Send (ns): " << duration_ns(stop - start) << "\t"; // Send takes roughly 81us
}

TEMPLATE_HEADER
MotorData MotorInterface<kServosPerChannel>::read_blocking(CANChannel bus)
{
    struct can_frame frame = read_canframe_blocking(bus);
    // cout << "CAN id: " << frame.can_id << endl;
    return parse_frame(frame);
}

TEMPLATE_HEADER
struct can_frame MOTOR_INTERFACE::read_canframe_blocking(CANChannel bus)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    // Print time to get fd
    auto start = time_now();
    int fd = canbus_to_fd_.at(static_cast<int>(bus));
    auto stop = time_now();
    // cout << "FD lookup (ns): " << duration_ns(stop - start) << "\t";

    // Print time to read from can bus
    auto start_read = time_now();
    int nbytes = read(fd, &frame, sizeof(struct can_frame));
    auto stop_read = time_now();
    // cout << "Read start (ms): " << duration_ms(start_read - debug_start_) << "\t";
    // cout << "Read end (ms): " << duration_ms(stop_read - debug_start_) << "\t";
    if (nbytes < 0)
    {
        // Continue on read timeouts
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            cout << "Bus " << static_cast<int>(bus) << " timed out on read" << endl;
        }
        cerr << "ERROR: can raw socket read";
    }
    if (nbytes != sizeof(struct can_frame))
    {
        cerr << "ERROR: did not read full can frame";
    }
    stop = time_now();
    // cout << "CAN read (ns): " << duration_ns(stop - start_read) << "\t";
    return frame;
}

TEMPLATE_HEADER
MotorData MOTOR_INTERFACE::parse_frame(const struct can_frame &frame)
{
    MotorData motor_data;
    motor_data.motor_id = motor_id(frame.can_id);
    if (motor_data.motor_id <= 0 || motor_data.motor_id > kServosPerChannel)
    {
        cout << "Invalid motor id." << endl;
        return motor_data;
    }
    uint8_t command_id = frame.data[0];
    if (command_id == kGetMultiAngle)
    {
        auto start = time_now();
        int64_t multi_loop_angle = 0;
        memcpy(&multi_loop_angle, frame.data, 8);
        multi_loop_angle = multi_loop_angle >> 8;
        auto stop = time_now();
        // cout << "Frame parse (ns): " << duration_ns(stop - start) << "\t";
        motor_data.multi_angle = multi_loop_angle * kDegsPerTick * kSpeedReduction;
    }
    if (command_id == kCommandCurrent || command_id == kCommandVelocity)
    {
        int8_t temp = static_cast<int8_t>(frame.data[1]);
        int16_t current_raw;
        memcpy(&current_raw, frame.data + 2, 2);
        int16_t speed_raw;
        memcpy(&speed_raw, frame.data + 4, 2);
        int16_t position_raw;
        memcpy(&position_raw, frame.data + 6, 2);

        motor_data.temp = temp;
        motor_data.current = (float)current_raw * kCurrentReadMax / kCurrentRawReadMax;
        motor_data.velocity = speed_raw;
        motor_data.encoder_position = position_raw;
    }
    return motor_data;
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::start_read_threads()
{
    for (auto canbus : motor_connections_)
    {
        read_threads_.at(static_cast<int>(canbus)) = thread(&MOTOR_INTERFACE::read_thread, this, canbus);
    }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_bus(CANChannel bus)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        cerr << "socket";
    }

    // find interface index
    strcpy(ifr.ifr_name, channel_str(bus).c_str());
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex)
    {
        cerr << "if_nametoindex";
    }
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    cout << channel_str(bus) << " ifindex: " << addr.can_ifindex << endl;

    // bind socket
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        cerr << "bind";
    }
    canbus_to_fd_.at(static_cast<int>(bus)) = s;

    // Set timeout
    struct timeval tv;
    tv.tv_sec = kTimeoutSeconds;
    tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_motor(CANChannel bus, uint32_t motor_id)
{
    send(bus, motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
    usleep(10000);
    send(bus, motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
    usleep(10000);
    send(bus, motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
    usleep(10000);
}

TEMPLATE_HEADER
uint32_t MOTOR_INTERFACE::can_id(uint32_t motor_id)
{
    return 0x140 + motor_id;
}

TEMPLATE_HEADER
uint32_t MOTOR_INTERFACE::motor_id(uint32_t can_id)
{
    return can_id - 0x140;
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::read_thread(CANChannel channel)
{
    while (should_read_.load())
    {
        // block until multi angle can message is read
        // MotorData motor_data = read_multi_angle(channel);
        MotorData motor_data = read_blocking(channel);
        if (!motor_data.error)
        {
            unique_lock<mutex> lock(latest_data_lock_);
            // TODO: think about how to copy data from can. Only copy new data so don't overwrite data from other types of commands
            // cout << "Motor data: " << (int)motor_data.motor_id << "\t" << motor_data.multi_angle << "\t";
            latest_data_.at(static_cast<int>(channel)).at(motor_data.motor_id - 1) = motor_data;
        }
    }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::send(CANChannel bus, uint32_t motor_id, const array<uint8_t, 8> &payload)
{
    int file_descriptor = canbus_to_fd_.at(static_cast<int>(bus));
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id(motor_id);
    frame.len = 8;
    memcpy(frame.data, payload.data(), 8);
    auto start = time_now();
    if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU)
    {
        cerr << "Error writing can frame";
    }
    auto stop = time_now();
    // cout << "write (ns): " << duration_ns(stop - start) << "\t"; // Takes around 80us
    // cout << "Send start (ms): " << duration_ms(start - debug_start_) << "\t";
    // cout << "Send end (ms): " << duration_ms(stop - debug_start_) << "\t";
}

template class MotorInterface<1>;
template class MotorInterface<2>;
template class MotorInterface<3>;
template class MotorInterface<4>;
template class MotorInterface<5>;
template class MotorInterface<6>;
