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

    std::cout << "Starting calibration..." << std::endl;
    controller.calibrate_motors(quit);

    std::cout << "blocking move" << std::endl;
    MotorController<K_SERVOS_PER_CHANNEL>::ActuatorCommand command = {{0, 0, 0, 0, 0, 0}};
    controller.blocking_move(quit,
                             /*max_speed(rotor deg/s)=*/1000.0,
                             /*speed_kp=*/10.0,
                             command,
                             /*speed_tolerance(output rad/s)=*/0.01,
                             /*wait_ticks=*/100);
    std::cout << "finished blocking move" << std::endl;
    return 0;
}
