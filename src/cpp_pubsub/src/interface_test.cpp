
#include <cmath>
#include <iostream>
#include <thread>
#include "gflags/gflags.h"
#include "motor_interface_generic.hpp"

DEFINE_bool(verbose_interface, false, "Verbose interface mode");
DEFINE_bool(verbose, false, "Verbose mode");
DEFINE_bool(velocity_control, false, "Test velocity control");

using namespace pupperv3;

std::atomic<bool> quit(false);

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

void test_command_velocity(bool verbose_interface, bool verbose) {
  MotorID id1{CANChannel::CAN0, 1};
  MotorID id2{CANChannel::CAN0, 2};
  MotorID id3{CANChannel::CAN0, 3};
  MotorID id4{CANChannel::CAN0, 4};
  MotorID id5{CANChannel::CAN0, 5};
  MotorID id6{CANChannel::CAN0, 6};

  ActuatorConfiguration actuator_config{{id1, id2, id3, id4, id5, id6}};
  MotorInterface interface(actuator_config, verbose_interface);
  interface.initialize_canbuses();
  interface.initialize_motors();
  interface.start_read_threads();

  auto start = std::chrono::high_resolution_clock::now();
  int num_sent = 0;
  while (!quit) {
    auto now = std::chrono::high_resolution_clock::now();
    auto us_since_start =
        std::chrono::duration_cast<std::chrono::microseconds>(start - now).count();
    double phase = 4 * us_since_start / 1000000.0;
    double value = std::sin(phase) * 2000;
    constexpr int sleep_us = 0;
    interface.command_velocity(id1, value * 1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    interface.command_velocity(id2, value * -1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    interface.command_velocity(id3, value * 1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    interface.command_velocity(id4, value * 1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    interface.command_velocity(id5, value * -1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    interface.command_velocity(id6, value * 1.0);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    num_sent++;
    auto counts = interface.receive_counts();
    std::cout << "sent: " << num_sent << " missing replies: ";
    for (int cnt : counts) {
      std::cout << num_sent - cnt << " ";
    }
    std::cout << "\n";
  }

  // for (int i = 0; i < 500; i++) {
  //   interface.command_velocity(id1, 2000);
  //   interface.command_velocity(id2, 8000);
  //   interface.command_velocity(id3, -10000);
  //   MotorInterface::ActuatorData data = interface.motor_data_safe();
  //   if (verbose) {
  //     std::cout << data << "\n";
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(2));
  //   if (quit) {
  //     break;
  //   }
  // }

  // for (int i = 0; i < 500; i++) {
  //   interface.command_velocity(id1, -2000);
  //   interface.command_velocity(id2, -8000);
  //   interface.command_velocity(id3, 10000);
  //   MotorInterface::ActuatorData data = interface.motor_data_safe();
  //   if (verbose) {
  //     std::cout << data << "\n";
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(2));
  //   if (quit) {
  //     break;
  //   }
  // }

  interface.command_velocity(id1, 0);
  interface.command_velocity(id2, 0);
  interface.command_velocity(id3, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  interface.command_all_stop();
}

int main(int argc, char* argv[]) {
  signal(SIGINT, sigint_handler);

  gflags::SetUsageMessage("USB2CAN pupper v3 generic interface test");
  gflags::SetVersionString("0.1");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_verbose) {
    std::cout << gflags::ProgramInvocationShortName() << ":\n";
  }

  if (FLAGS_velocity_control) {
    test_command_velocity(FLAGS_verbose_interface, FLAGS_verbose);
  }

  gflags::ShutDownCommandLineFlags();
  return 0;
};