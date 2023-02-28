
#include <iostream>
#include <thread>
#include "gflags/gflags.h"
#include "motor_interface_generic.hpp"

DEFINE_bool(verbose, false, "Verbose mode");
DEFINE_bool(velocity_control, false, "Test velocity control");

using namespace pupperv3;

std::atomic<bool> quit(false);

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

void test_command_velocity() {
  MotorID id1{CANChannel::CAN0, 1};
  MotorID id2{CANChannel::CAN0, 2};
  MotorID id3{CANChannel::CAN0, 3};

  ActuatorConfiguration actuator_config{{id1, id2, id3}};
  MotorInterface interface(actuator_config, FLAGS_verbose);
  interface.initialize_canbuses();
  interface.initialize_motors();
  interface.start_read_threads();

  for (int i = 0; i < 500; i++) {
    interface.command_velocity(id1, 1000);
    interface.command_velocity(id2, 2000);
    interface.command_velocity(id3, 4000);
    MotorInterface::ActuatorData data = interface.motor_data_safe();
    std::cout << data << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (quit) {
      break;
    }
  }

  for (int i = 0; i < 500; i++) {
    interface.command_velocity(id1, -1000);
    interface.command_velocity(id2, -2000);
    interface.command_velocity(id3, -4000);
    MotorInterface::ActuatorData data = interface.motor_data_safe();
    std::cout << data << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (quit) {
      break;
    }
  }

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
    test_command_velocity();
  }

  gflags::ShutDownCommandLineFlags();
  return 0;
};