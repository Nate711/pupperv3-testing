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

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std;

// Combine with the kchannelloopup by making a class?
enum class CANChannel
{
    CAN0 = 0,
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

unordered_map<CANChannel, string> kChannelLookup = {{CANChannel::CAN0, "can0"}, {CANChannel::CAN1, "can1"}};

class MotorInterface
{
public:
    MotorInterface(unordered_map<CANChannel, vector<uint32_t>> motor_connections, int bitrate) : motor_connections_(motor_connections),
                                                                                                 bitrate_(bitrate),
                                                                                                 initialized_(false)
    {
    }

    ~MotorInterface()
    {
        close_canbuses();
    }

    void initialize_canbuses()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            initialize_bus(bus);
        }
    }

    void close_canbuses()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            close(canbus_to_fd_.at(static_cast<int>(bus)));
        }
        cout << "Closed can bus sockets." << endl;
    }

    void initialize_motors()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            for (const auto &motor_id : motor_id_list)
            {
                cout << "init motor id: " << motor_id << endl;
                initialize_motor(bus, motor_id);
            }
        }
        initialized_ = true;
    }

    float get_multi_angle(CANChannel bus, uint32_t motor_id)
    {

        auto start = chrono::high_resolution_clock::now();
        send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
        auto stop = chrono::high_resolution_clock::now();
        cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        start = chrono::high_resolution_clock::now();
        int fd = canbus_to_fd_.at(static_cast<int>(bus));
        stop = chrono::high_resolution_clock::now();
        cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

        start = chrono::high_resolution_clock::now();
        int nbytes = read(fd, &frame, sizeof(struct can_frame));
        stop = chrono::high_resolution_clock::now();
        cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

        start = chrono::high_resolution_clock::now();
        if (nbytes < 0)
        {
            cerr << "can raw socket read";
        }
        // cout << "frame data: " << frame.data << endl;
        int64_t multi_loop_angle;
        memcpy(&multi_loop_angle, frame.data + 1, 7);
        stop = chrono::high_resolution_clock::now();
        cout << chrono::duration_cast<chrono::nanoseconds>(stop - start).count() << "\t";

        return multi_loop_angle * kDegsPerTick * kSpeedReduction;
    }

private:
    void initialize_bus(CANChannel bus)
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
        cout << "can ifindex: " << addr.can_ifindex << endl;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            cerr << "bind";
        }
        canbus_to_fd_.at(static_cast<int>(bus)) = s;
    }

    void initialize_motor(CANChannel bus, uint32_t motor_id)
    {
        send(bus, motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
        send(bus, motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
        send(bus, motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
    }

    uint32_t can_id(uint32_t motor_id)
    {
        return 0x140 + motor_id;
    }

    void send(CANChannel bus, uint32_t motor_id, array<uint8_t, 8> payload)
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

    // unordered_map<CANChannel, int> canbus_to_fd_;
    array<int, 4> canbus_to_fd_;
    unordered_map<CANChannel, vector<uint32_t>> motor_connections_;
    int bitrate_;
    bool initialized_;
    uint8_t kStartup0 = 0x76;
    uint8_t kStartup1 = 0x88;
    uint8_t kStartup2 = 0x77;
    uint8_t kGetMultiAngle = 0x92;
    float kDegsPerTick = 0.01;
    float kSpeedReduction = 0.1;
};

// TODO: figure out how to catch control-c and close sockets without this global
unique_ptr<MotorInterface> motor_interface;

// TODO: figure out better way to catch control-c
void sigint_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n", s);
    exit(1);
}

int main()
{
    signal(SIGINT, sigint_handler);

    // TODO: Replace with make_unique after resolving argument list error
    motor_interface = unique_ptr<MotorInterface>(new MotorInterface({{CANChannel::CAN0, {1}}}, /*bitrate=*/1000000));
    motor_interface->initialize_canbuses();
    motor_interface->initialize_motors();

    auto loop_start = chrono::high_resolution_clock::now();
    while (true)
    {
        auto loop_now = chrono::high_resolution_clock::now();
        auto since_start = chrono::duration_cast<chrono::microseconds>(loop_now - loop_start);
        cout << since_start.count() << "\t";

        auto start = chrono::high_resolution_clock::now();
        float angle = motor_interface->get_multi_angle(CANChannel::CAN0, 1);
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::nanoseconds>(stop - start);
        // cout << "angle: " << angle << endl;
        cout << duration.count() << endl;
    }
    return 0;
}
