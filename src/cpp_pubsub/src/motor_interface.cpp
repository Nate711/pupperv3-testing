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

MotorInterface::MotorInterface(unordered_map<CANChannel, vector<uint32_t>> motor_connections, int bitrate) : motor_connections_(motor_connections),
                                                                                                             bitrate_(bitrate),
                                                                                                             initialized_(false),
                                                                                                             should_read_(true)
{
    // DEBUG ONLY
    debug_start_ = time_now();
    canbus_to_fd_.fill(-1);
    initialize_map();
}

MotorInterface::~MotorInterface()
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

void MotorInterface::initialize_map()
{
    for (const auto &[bus, motor_id_list] : motor_connections_)
    {
        for (const auto &motor_id : motor_id_list)
        {
            latest_data_.at(static_cast<int>(bus)).insert({motor_id, MotorData{.motor_id = motor_id}});
        }
    }
}

void MotorInterface::initialize_canbuses()
{
    for (const auto &[bus, motor_id_list] : motor_connections_)
    {
        initialize_bus(bus);
    }
}

void MotorInterface::close_canbuses()
{
    for (const auto &[bus, motor_id_list] : motor_connections_)
    {
        close(canbus_to_fd_.at(static_cast<int>(bus)));
    }
    cout << "Closed can bus sockets." << endl;
}

void MotorInterface::initialize_motors()
{
    for (const auto &[bus, motor_id_list] : motor_connections_)
    {
        for (const auto &motor_id : motor_id_list)
        {
            initialize_motor(bus, motor_id);
        }
    }
    initialized_ = true;
}

array<unordered_map<int, MotorData>, kNumCANChannels> MotorInterface::latest_data()
{
    {
        unique_lock<mutex> lock(latest_data_lock_);
        return latest_data_;
    }
}

void MotorInterface::request_multi_angle(CANChannel bus, uint32_t motor_id)
{
    auto start = time_now();
    send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
    auto stop = time_now();
    // cout << "Send (ns): " << duration_ns(stop - start) << "\t"; // Send takes roughly 81us
}

MotorData MotorInterface::read_multi_angle(CANChannel bus)
{
    struct MotorData motor_data;
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
            return MotorData{.error=1};
        }
        cerr << "ERROR: can raw socket read";
    }
    if (nbytes != sizeof(struct can_frame))
    {
        cerr << "ERROR: did not read full can frame";
    }
    stop = time_now();
    // cout << "CAN read (ns): " << duration_ns(stop - start_read) << "\t";

    // Print time to copy data into angle
    start = time_now();
    uint8_t command_id = frame.data[0];
    int64_t multi_loop_angle = 0;
    memcpy(&multi_loop_angle, frame.data, 8);
    multi_loop_angle &= 0xFFFFFFFFFFFFFF00;
    multi_loop_angle = multi_loop_angle >> 8;
    stop = time_now();
    // cout << "Frame parse (ns): " << duration_ns(stop - start) << "\t";

    motor_data.multi_angle = multi_loop_angle * kDegsPerTick * kSpeedReduction;
    motor_data.motor_id = motor_id(frame.can_id);
    return motor_data;
}

void MotorInterface::start_read_threads()
{
    for (auto canbus : kAllCANChannels)
    {
        if (canbus_to_fd_.at(static_cast<int>(canbus)) != -1)
        {
            read_threads_.at(static_cast<int>(canbus)) = thread(&MotorInterface::read_thread, this, canbus);
        }
    }
}

void MotorInterface::initialize_bus(CANChannel bus)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        cerr << "socket";
    }

    // find interface index
    strcpy(ifr.ifr_name, kChannelLookup[bus].c_str());
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex)
    {
        cerr << "if_nametoindex";
    }
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    cout << kChannelLookup[bus] << " ifindex: " << addr.can_ifindex << endl;

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

void MotorInterface::initialize_motor(CANChannel bus, uint32_t motor_id)
{
    send(bus, motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
    send(bus, motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
    send(bus, motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
}

uint32_t MotorInterface::can_id(uint32_t motor_id)
{
    return 0x140 + motor_id;
}

uint32_t MotorInterface::motor_id(uint32_t can_id)
{
    return can_id - 0x140;
}

void MotorInterface::read_thread(CANChannel channel)
{
    while (should_read_.load())
    {
        // block until multi angle can message is read
        MotorData motor_data = read_multi_angle(channel);
        if (!motor_data.error)
        {
            unique_lock<mutex> lock(latest_data_lock_);
            // TODO: think about how to copy data from can. Only copy new data
            // cout << "Motor data: " << (int)motor_data.motor_id << "\t" << motor_data.multi_angle << "\t";
            latest_data_.at(static_cast<int>(channel)).at(motor_data.motor_id).multi_angle = motor_data.multi_angle;
        }
    }
}

void MotorInterface::send(CANChannel bus, uint32_t motor_id, const array<uint8_t, 8> &payload)
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