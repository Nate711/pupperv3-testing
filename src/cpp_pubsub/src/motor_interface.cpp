#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <cstring>
#include <array>
#include <map>
#include <signal.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class MotorInterface
{
public:
    MotorInterface(std::map<std::string, std::vector<uint32_t>> motor_connections, int bitrate) : motor_connections_(motor_connections),
                                                                                                  bitrate_(bitrate),
                                                                                                  initialized_(false)
    {
    }

    ~MotorInterface()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            close(canbus_to_fd_[bus]);
        }
    }

    void initialize_canbuses()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            initialize_bus(bus);
        }
    }

    void initialize_motors()
    {
        for (const auto &[bus, motor_id_list] : motor_connections_)
        {
            for (const auto &motor_id : motor_id_list)
            {
                std::cout << "init motor id: " << motor_id << std::endl;
                initialize_motor(bus, motor_id);
            }
        }
        initialized_ = true;
    }

    float get_multi_angle(std::string bus, uint32_t motor_id)
    {
        send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        int fd = canbus_to_fd_[bus];
        int nbytes = read(fd, &frame, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            std::cerr << "can raw socket read";
        }
        std::cout << "frame data: " << frame.data << std::endl;
        int64_t multi_loop_angle;
        std::memcpy(&multi_loop_angle, frame.data + 1, 7);
        return multi_loop_angle * kDegsPerTick * kSpeedReduction;
    }

private:
    void initialize_bus(std::string bus)
    {
        int s;
        struct sockaddr_can addr;
        struct ifreq ifr;
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            std::cerr << "socket";
        }
        strcpy(ifr.ifr_name, bus.c_str());
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
        if (!ifr.ifr_ifindex)
        {
            std::cerr << "if_nametoindex";
        }
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        std::cout << "can ifindex: " << addr.can_ifindex << std::endl;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            std::cerr << "bind";
        }
        canbus_to_fd_[bus] = s;
    }

    void initialize_motor(std::string bus, uint32_t motor_id)
    {
        send(bus, motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
        send(bus, motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
        send(bus, motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
    }

    uint32_t can_id(uint32_t motor_id)
    {
        return 0x140 + motor_id;
    }

    void send(std::string bus, uint32_t motor_id, std::array<uint8_t, 8> payload)
    {
        int file_descriptor = canbus_to_fd_[bus];
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id(motor_id);
        frame.len = 8;
        std::copy(payload.begin(), payload.end(), frame.data);
        if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU)
        {
            std::cerr << "Error writing can frame";
        }
    }
    std::map<std::string, int> canbus_to_fd_;
    std::map<std::string, std::vector<uint32_t>> motor_connections_;
    int bitrate_;
    bool initialized_;
    uint8_t kStartup0 = 0x76;
    uint8_t kStartup1 = 0x88;
    uint8_t kStartup2 = 0x77;
    uint8_t kGetMultiAngle = 0x92;
    float kDegsPerTick = 0.01;
    float kSpeedReduction = 0.1;
};

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n", s);
    exit(1);
}

int main()
{
    signal(SIGINT, my_handler);

    MotorInterface motor_interface({{"can0", {1}}}, /*bitrate=*/1000000);
    motor_interface.initialize_canbuses();
    motor_interface.initialize_motors();

    while (true)
    {
        float angle = motor_interface.get_multi_angle("can0", 1);
        std::cout << "angle: " << angle << std::endl;
        sleep(0.005);
    }

    // int s;
    // int required_mtu;
    // struct can_frame frame;
    // struct sockaddr_can addr;
    // struct ifreq ifr;
    // if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    // {
    //     std::cerr << "socket";
    // }
    // // strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
    // // ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    // strcpy(ifr.ifr_name, "can0");

    // ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    // if (!ifr.ifr_ifindex)
    // {
    //     std::cerr << "if_nametoindex";
    // }
    // memset(&addr, 0, sizeof(addr));
    // addr.can_family = AF_CAN;
    // addr.can_ifindex = ifr.ifr_ifindex;
    // std::cout << addr.can_ifindex << std::endl;

    // if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    // {
    //     std::cerr << "bind";
    // }

    // memset(&frame, 0, sizeof(frame));
    // frame.can_id = 0x0141;
    // frame.len = 8;
    // frame.data[0] = 0x92;

    // if (write(s, &frame, CAN_MTU) != CAN_MTU)
    // {
    //     std::cerr << "write";
    // }

    // close(s);

    //   rclcpp::init(argc, argv);
    //   rclcpp::spin(std::make_shared<MinimalPublisher>());
    //   rclcpp::shutdown();
    return 0;
}
