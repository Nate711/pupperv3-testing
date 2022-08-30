#include "motor_controller.hpp"
#include "prof_utils.hpp"
#include <atomic>
#include <memory>
#include <algorithm>
#include <math.h>

#define K_SERVOS_PER_CHANNEL 6
#define PRINT_CYCLE 1

atomic<bool> quit(false); // signal flag

void sigint_handler(sig_atomic_t s)
{
    quit.store(true);
    printf("Caught signal %d\n", s);
}

int main(int argc, char *argv[])
{
    // Boilerplate
    int sleep_us = 2000;
    if (argc > 1)
    {
        sleep_us = std::stoi(argv[1]);
    }
    signal(SIGINT, sigint_handler);

    // Create position controller
    MotorController<6> controller(/*kp (dps per deg)=*/50.0,
                                  /*max_speed (dps)=*/20000.0,
                                  /*can channels*/ {CANChannel::CAN0},
                                  /*bitrate=*/1000000);
    controller.begin();
    cout << "Initialized controller." << endl;
    vector<array<float, 6>> goal_positions = {{0, 0, 0, 0, 0, 0}};

    // Boiler plate
    auto loop_start = time_now();
    int loop_count = 0;
    auto last_loop_ts = time_now();
    while (!quit.load())
    {
        auto loop_now = time_now();
        if (loop_count % PRINT_CYCLE == 0)
        {
            cout << "\nSince start (us): " << duration_ms(loop_now - loop_start) << "\t";
            cout << "DT (us): " << duration_ms(loop_now - last_loop_ts) << "\t";
        }
        last_loop_ts = loop_now;

        float amp = 450.0; // 90 deg sweep at output
        float secs_since_start = duration_ms(loop_now - loop_start) / 1000000.0;
        float freq = secs_since_start / 2.0; // chirp signal
        float sample = std::sin(secs_since_start * 2 * M_PI * freq);
        // Set desired positions
        goal_positions.at(0).at(0) = amp * sample;
        controller.run(goal_positions);

        if (loop_count % PRINT_CYCLE == 0)
        {
            auto common = controller.motor_data_copy(CANChannel::CAN0, 1).common;
            cout << common.current << "\t" << common.multi_loop_angle << "\t";
        }

        usleep(sleep_us);
        loop_count++;
    }
    return 0;
}
