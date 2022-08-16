#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <array>
// #include <map>
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

using namespace std;

MotorInterface::MotorInterface(unordered_map<CANChannel, vector<uint32_t>> motor_connections, int bitrate) : motor_connections_(motor_connections),
                                                                                                             bitrate_(bitrate),
                                                                                                             initialized_(false)
{
}

MotorInterface::~MotorInterface()
{
    close_canbuses();
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

void MotorInterface::request_multi_angle(CANChannel bus, uint32_t motor_id)
{
    auto start = chrono::high_resolution_clock::now();
    send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
    auto stop = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";
}

float MotorInterface::read_multi_angle(CANChannel bus)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    auto start = chrono::high_resolution_clock::now();
    int fd = canbus_to_fd_.at(static_cast<int>(bus));
    auto stop = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

    start = chrono::high_resolution_clock::now();
    int nbytes = read(fd, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
        cerr << "ERROR: can raw socket read";
    }
    if (nbytes != sizeof(struct can_frame))
    {
        cerr << "ERROR: did not read full can frame";
    }
    stop = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

    start = chrono::high_resolution_clock::now();
    // cout << "frame data: " << frame.data << endl;
    int64_t multi_loop_angle;
    memcpy(&multi_loop_angle, frame.data + 1, 7);
    stop = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

    return multi_loop_angle * kDegsPerTick * kSpeedReduction;
}

void MotorInterface::start_read_thread()
{
    read_multi_angle_thread_ = make_unique<thread>(&MotorInterface::read_multi_angle_thread, this);
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

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        cerr << "bind";
    }
    canbus_to_fd_.at(static_cast<int>(bus)) = s;
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

void MotorInterface::read_multi_angle_thread()
{
    while (true)
    {
        float angle = read_multi_angle(CANChannel::CAN0);
        {
            unique_lock<mutex> lock(latest_multi_angle_mutex_);
            latest_multi_angle_ = angle;
        }
        cout << "got angle: " << angle << endl;
    }
}

void MotorInterface::send(CANChannel bus, uint32_t motor_id, const array<uint8_t, 8> &payload)
{
    int file_descriptor = canbus_to_fd_.at(static_cast<int>(bus));
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id(motor_id);
    frame.len = 8;
    copy(payload.begin(), payload.end(), frame.data);
    if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU)
    {
        cerr << "Error writing can frame";
    }
}
