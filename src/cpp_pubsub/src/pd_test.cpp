#include "motor_interface.hpp"
#include "prof_utils.hpp"
#include <atomic>
#include <memory>
#include <algorithm>

#define K_SERVOS_PER_CHANNEL 6
#define PRINT_CYCLE 1

std::atomic<bool> quit(false); // signal flag

void sigint_handler(sig_atomic_t s)
{
    quit.store(true);
    printf("Caught signal %d\n", s);
}

int main()
{
    signal(SIGINT, sigint_handler);

    // TODO: Replace with make_unique after resolving argument list error
    MotorInterface motor_interface = MotorInterface<K_SERVOS_PER_CHANNEL>({CANChannel::CAN0});
    motor_interface.initialize_canbuses();
    motor_interface.initialize_motors(); // not needed if you just want angles
    motor_interface.start_read_threads();
    std::cout << "Initialized canbuses, motors, threads" << std::endl;

    auto loop_start = time_now();

    int loop_count = 0;
    auto last_loop_ts = time_now();
    while (!quit.load())
    {
        // Print time since start of program
        auto loop_now = time_now();
        if (loop_count % PRINT_CYCLE == 0)
        {
            std::cout << "\nSince start (us): " << duration_ms(loop_now - loop_start) << "\t";
            std::cout << "DT (us): " << duration_ms(loop_now - last_loop_ts) << "\t";
        }
        last_loop_ts = loop_now;

        auto motor_data = motor_interface.motor_data_safe(CANChannel::CAN0, 1);

        motor_interface.request_multi_angle(CANChannel::CAN0, 1); // would overwrite motor_data
        motor_interface.request_multi_angle(CANChannel::CAN0, 2);
        motor_interface.request_multi_angle(CANChannel::CAN0, 3);
        motor_interface.request_multi_angle(CANChannel::CAN0, 4);
        motor_interface.request_multi_angle(CANChannel::CAN0, 5);
        motor_interface.request_multi_angle(CANChannel::CAN0, 6);

        float position_target = 1000.0;
        float position_error = position_target - motor_data.common.multi_loop_angle;
        float velocity_command = position_error * 10;
        velocity_command = std::clamp(velocity_command, -200.0f, 200.0f);
        motor_interface.command_velocity(CANChannel::CAN0, 1, velocity_command);
        if (loop_count % PRINT_CYCLE == 0)
        {
            // cout << motor_data.temp << "\t" << motor_data.current << "\t" << motor_data.velocity << "\t" << motor_data.encoder_counts << "\t" << rotations << "\t" << multi_loop_angle << "\t";
            std::cout << motor_data.common.current << "\t" << motor_data.common.velocity_degs << "\t" << motor_data.common.encoder_counts << "\t";
            std::cout << motor_data.common.rotations << "\t" << motor_data.common.multi_loop_angle << "\t";

            for (int motor_id = 1; motor_id < K_SERVOS_PER_CHANNEL; motor_id++)
            {
                std::cout << motor_interface.motor_data_safe(CANChannel::CAN0, motor_id).multi_loop.multi_loop_angle;
            }
        }
        // motor_interface.request_multi_angle(CANChannel::CAN0, 1);
        // motor_interface.command_velocity(CANChannel::CAN0, 1, 0.0);
        usleep(2000); // sending to 3 motors takes 2500us

        // torque control: jiggles with kd=0.01 and usleep(5000)
        // torque control: jiggles with kd=0.01 and usleep(2000)
        // speed-based pos control: good with kp = 10, usleep(1000 or 2000), max speed 3600
        // works at least at 1khz, 500hz, 200hz
        // have to plug in motor before running program
        loop_count++;
    }
    return 0;
}
