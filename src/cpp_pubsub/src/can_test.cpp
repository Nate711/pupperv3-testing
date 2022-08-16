#include "motor_interface.hpp"

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

    motor_interface->start_read_thread();

    auto loop_start = chrono::high_resolution_clock::now();
    while (true)
    {
        auto loop_now = chrono::high_resolution_clock::now();
        auto since_start = chrono::duration_cast<chrono::microseconds>(loop_now - loop_start);
        cout << since_start.count() << "\t";

        auto start = chrono::high_resolution_clock::now();
        motor_interface->request_multi_angle(CANChannel::CAN0, 1);
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::nanoseconds>(stop - start);
        // cout << "angle: " << angle << endl;
        cout << duration.count() << endl;
    }
    return 0;
}
