#include "motor_interface.hpp"
#include "prof_utils.hpp"
#include <atomic>
#include <memory>

#define K_SERVOS_PER_CHANNEL 1

atomic<bool> quit(false); // signal flag

void sigint_handler(sig_atomic_t s)
{
    quit.store(true);
    printf("Caught signal %d\n", s);
}

int main()
{
    signal(SIGINT, sigint_handler);

    // TODO: Replace with make_unique after resolving argument list error
    MotorInterface motor_interface = MotorInterface<K_SERVOS_PER_CHANNEL>({CANChannel::CAN0}, /*bitrate=*/1000000);
    motor_interface.initialize_canbuses();
    motor_interface.initialize_motors(); // not needed if you just want angles
    motor_interface.start_read_threads();
    cout << "Initialized canbuses, motors, threads" << endl;

    auto loop_start = time_now();
    while (!quit.load())
    {
        // Print time since start of program
        auto loop_now = time_now();
        auto since_start = chrono::duration_cast<chrono::microseconds>(loop_now - loop_start);
        cout << "\nSince start (us): " << since_start.count() << "\t";

        // Print time took to send angle request
        // motor_interface.command_current(/*channel=*/CANChannel::CAN0, /*motor_id=*/1, /*current=*/0.0);
        // motor_interface.request_multi_angle(CANChannel::CAN0, 1);
        motor_interface.command_velocity(CANChannel::CAN0, 1, 360.0);
        usleep(5000); // sending to 3 motors takes 2500us

        auto latest_data = motor_interface.latest_data();
        auto motor_data = latest_data.at(0).at(0);
        cout << motor_data.temp << "\t" << motor_data.current << "\t" << motor_data.velocity << "\t" << motor_data.encoder_position << "\t";
    }
    return 0;
}
