#include <atomic>
#include "motor_interface.hpp"
#include "prof_utils.hpp"

#define K_SERVOS_PER_CHANNEL 3

std::atomic<bool> quit(false);  // signal flag

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

int main() {
  signal(SIGINT, sigint_handler);

  // TODO: Replace with make_unique after resolving argument list error
  auto motor_interface = std::unique_ptr<MotorInterface<K_SERVOS_PER_CHANNEL>>(
      new MotorInterface<K_SERVOS_PER_CHANNEL>({CANChannel::CAN0, CANChannel::CAN1}));
  motor_interface->initialize_canbuses();
  motor_interface->initialize_motors();  // not needed if you just want angles
  motor_interface->start_read_threads();
  std::cout << "Initialized canbuses, motors, threads" << std::endl;

  float prevent_cut = 0;

  auto loop_start = time_now();
  while (!quit.load()) {
    // Print time since start of program
    auto loop_now = time_now();
    auto since_start = std::chrono::duration_cast<std::chrono::microseconds>(loop_now - loop_start);
    std::cout << "\nSince start (us): " << since_start.count() << "\t";

    // Print time took to send angle request
    auto start = time_now();
    motor_interface->request_multi_angle(CANChannel::CAN0, 1);
    motor_interface->request_multi_angle(CANChannel::CAN0, 2);
    motor_interface->request_multi_angle(CANChannel::CAN0, 3);
    // motor_interface->request_multi_angle(CANChannel::CAN1, 1);
    // motor_interface->request_multi_angle(CANChannel::CAN1, 2);
    // motor_interface->request_multi_angle(CANChannel::CAN1, 3);
    auto stop = time_now();
    std::cout << "Angle request (ns): " << duration_ns(stop - start)
              << "\t";  // takes 80us with large variance

    auto retrieve_start = time_now();
    auto latest_data = motor_interface->motor_data_safe();
    for (int bus = 0; bus < 2; bus++) {
      for (int servo = 0; servo < 3; servo++) {
        std::cout << latest_data.at(bus).at(servo).multi_loop.multi_loop_angle << "\t";
      }
    }
    auto retrieve_end = time_now();
    std::cout << "Retrival time (ns):\t" << duration_ns(retrieve_end - retrieve_start)
              << "\t";  // 3000ns average when adding angles
    std::cout << "Size of data: " << sizeof(latest_data) << std::endl;

    usleep(2000);  // sending to 3 motors takes 2500us
  }
  std::cout << prevent_cut << std::endl;
  return 0;
}
