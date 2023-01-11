#include <math.h>
#include <algorithm>
#include <atomic>
#include <memory>
#include "motor_controller.hpp"
#include "prof_utils.hpp"

using namespace std::chrono_literals;

#define K_SERVOS_PER_CHANNEL 6
#define PRINT_CYCLE 1

std::atomic<bool> quit(false);  // signal flag

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

int main(int argc, char *argv[]) {
  // Boilerplate
  signal(SIGINT, sigint_handler);

  // Create position controller
  MotorController<K_SERVOS_PER_CHANNEL> controller(
      /*position_kp (dps per output rad)=*/50000.0,
      /*speed_kp=*/5,
      /*max_speed (dps)=*/5000.0,
      /*can channels*/ {CANChannel::CAN0, CANChannel::CAN1});
  controller.begin();
  std::cout << "Initialized controller." << std::endl;

  controller.calibrate_motors(quit);

  MotorController<K_SERVOS_PER_CHANNEL>::ActuatorMatrix<float> command = {{0, 0, 1.0, 0, 0, -1.0},
                                                                          {0, 0, 1.0, 0, 0, -1.0}};
  controller.blocking_move(quit,
                           /*max_speed(rotor deg/s)=*/1000.0,
                           /*position_kp=*/20000.0,
                           /*speed_kp=*/10, command,
                           /*speed_tolerance(output rad/s)=*/0.01,
                           /*wait_ticks=*/50);
  return 0;
}
