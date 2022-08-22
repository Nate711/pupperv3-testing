#include "motor_interface.hpp"
#include "prof_utils.hpp"
#include <atomic>

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
    auto motor_interface = unique_ptr<MotorInterface>(new MotorInterface({{CANChannel::CAN0, {1, 2, 3}}, {CANChannel::CAN1, {1, 2, 3}}}, /*bitrate=*/1000000));
    motor_interface->initialize_canbuses();
    motor_interface->initialize_motors(); // not needed if you just want angles
    motor_interface->start_read_threads();
    cout << "Initialized canbuses, motors, threads" << endl;

    auto loop_start = time_now();
    while (!quit.load())
    {
        // Print time since start of program
        auto loop_now = time_now();
        auto since_start = chrono::duration_cast<chrono::microseconds>(loop_now - loop_start);
        cout << endl
             << "\nSince start (us): " << since_start.count() << "\t";

        // Print time took to send angle request
        auto start = time_now();
        motor_interface->request_multi_angle(CANChannel::CAN0, 1);
        motor_interface->request_multi_angle(CANChannel::CAN0, 2);
        motor_interface->request_multi_angle(CANChannel::CAN0, 3);
        motor_interface->request_multi_angle(CANChannel::CAN1, 1);
        motor_interface->request_multi_angle(CANChannel::CAN1, 2);
        motor_interface->request_multi_angle(CANChannel::CAN1, 3);
        auto stop = time_now();
        // cout << "Angle request (ns): " << duration_ns(stop - start) << "\t"; // takes 80us with large variance

        auto retrieve_start = time_now();
        cout << "Angles (deg): " << "\t";
        auto latest_data = motor_interface->latest_data();
        cout << latest_data.at(0).at(0).multi_angle << "\t";
        cout << "Size of data: " << sizeof(latest_data) << "\t";
        // cout << latest_data.at(0).at(1).multi_angle << "\t";
        // cout << latest_data.at(0).at(2).multi_angle << "\t";
        // cout << latest_data.at(1).at(0).multi_angle << "\t";
        // cout << latest_data.at(1).at(1).multi_angle << "\t";
        // cout << latest_data.at(1).at(2).multi_angle << "\t";
        auto retrieve_end = time_now();
        cout << "Retrival time (ns): " << duration_ns(retrieve_end - retrieve_start) << "\t"; // 30us to 300us, 12us to 80us

        // float angle = motor_interface->read_multi_angle(CANChannel::CAN0);
        // cout << "Multi-angle (deg): " << angle << "\t";

        // usleep(500-220);
        usleep(2000); // sending to 3 motors takes 2500us
    }
    return 0;
}
