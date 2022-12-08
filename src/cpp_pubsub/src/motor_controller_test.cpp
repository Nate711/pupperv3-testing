#include "motor_controller.hpp"
#include <math.h>
#include <algorithm>
#include <atomic>
#include <memory>
#include "prof_utils.hpp"

#define K_SERVOS_PER_CHANNEL 6
#define PRINT_CYCLE 1

std::atomic<bool> quit(false);  // signal flag

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

int main(int argc, char *argv[]) {
  // Boilerplate
  int sleep_us = 2000;
  if (argc > 1) {
    sleep_us = std::stoi(argv[1]);
  }
  signal(SIGINT, sigint_handler);

  // Create position controller
  std::shared_ptr<MotorController<K_SERVOS_PER_CHANNEL>> controller(
      new MotorController<K_SERVOS_PER_CHANNEL>(/*position_kp (dps per output rad)=*/50000.0,
                                                /*speed_kp=*/4,
                                                /*max_speed (dps)=*/5000.0,
                                                /*can channels*/ {CANChannel::CAN0}));
  controller->begin();
  std::cout << "Initialized controller." << std::endl;
  std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> goal_positions = {{0}};

  // Boiler plate
  auto loop_start = time_now();
  int loop_count = 0;
  auto last_loop_ts = time_now();
  while (!quit.load()) {
    auto loop_now = time_now();
    if (loop_count % PRINT_CYCLE == 0) {
      std::cout << "\nSince start (us): " << duration_ms(loop_now - loop_start) << "\t";
      std::cout << "DT (us): " << duration_ms(loop_now - last_loop_ts) << "\t";
    }
    last_loop_ts = loop_now;

    float amp = 0.785;  // 90 deg sweep at output
    float secs_since_start = duration_ms(loop_now - loop_start) / 1000000.0;
    // float freq = secs_since_start / 2.0; // chirp signal
    float freq = 2.0;
    float sample = std::sin(secs_since_start * 2 * M_PI * freq);
    // Set desired positions
    goal_positions.at(0).at(0) = amp * sample;
    controller->position_control(goal_positions);

    if (loop_count % PRINT_CYCLE == 0) {
      auto current = controller->actuator_efforts().at(0).at(0);
      auto pos = controller->actuator_positions().at(0).at(0);
      std::cout << "I(A): " << current << "\tOutput rads: " << pos << "\t";
    }

    usleep(sleep_us);
    loop_count++;
  }
  return 0;
}
