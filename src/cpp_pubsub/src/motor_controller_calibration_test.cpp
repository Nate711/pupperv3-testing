#include "motor_controller.hpp"
#include "prof_utils.hpp"
#include <atomic>
#include <memory>
#include <algorithm>
#include <math.h>

using namespace std::chrono_literals;

#define K_SERVOS_PER_CHANNEL 6
#define PRINT_CYCLE 1

std::atomic<bool> quit(false); // signal flag

void sigint_handler(sig_atomic_t s)
{
    quit.store(true);
    printf("Caught signal %d\n", s);
}

int main(int argc, char *argv[])
{
    // Boilerplate
    signal(SIGINT, sigint_handler);

    // Create position controller
    MotorController<K_SERVOS_PER_CHANNEL> controller(/*position_kp (dps per output rad)=*/50000.0,
                                                     /*speed_kp=*/4,
                                                     /*max_speed (dps)=*/5000.0,
                                                     /*can channels*/ {CANChannel::CAN0});
    controller.begin();
    std::cout << "Initialized controller." << std::endl;
    std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> goal_positions = {{0}};

    std::cout << "Starting calibration thread" << std::endl;
    controller.start_calibration();

    // Boiler plate
    auto loop_start = time_now();
    int loop_count = 0;
    auto last_loop_ts = time_now();
    while (!quit.load())
    {
        auto loop_now = time_now();
        if (loop_count % PRINT_CYCLE == 0)
        {
            std::cout << "Since start (us): " << duration_ms(loop_now - loop_start) << "\t";
            std::cout << "DT (us): " << duration_ms(loop_now - last_loop_ts) << "\t";
            std::cout << std::endl;
        }
        last_loop_ts = loop_now;

        std::this_thread::sleep_for(2000us);
        loop_count++;
    }
    return 0;
}
