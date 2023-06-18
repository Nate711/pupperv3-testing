#include <math.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <atomic>
#include <memory>
#include "motor_controller_generic.hpp"
#include "prof_utils.hpp"

using namespace std::chrono_literals;

#define N_ACTUATORS 6
#define PRINT_CYCLE 1

std::atomic<bool> quit(false);  // signal flag

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

int main(int argc, char *argv[]) {
  spdlog::set_pattern("[%H:%M:%S.%e] [%^%7l%$] [%35s:%#] %^%v%$");
  signal(SIGINT, sigint_handler);

  using ActuatorVector = pupperv3::MotorController<N_ACTUATORS>::ActuatorVector;

  // Create config
  pupperv3::MotorID id1{pupperv3::CANChannel::CAN0, 1};
  pupperv3::MotorID id2{pupperv3::CANChannel::CAN0, 2};
  pupperv3::MotorID id3{pupperv3::CANChannel::CAN0, 3};
  pupperv3::MotorID id4{pupperv3::CANChannel::CAN0, 4};
  pupperv3::MotorID id5{pupperv3::CANChannel::CAN0, 5};
  pupperv3::MotorID id6{pupperv3::CANChannel::CAN0, 6};
  // pupperv3::ActuatorConfiguration actuator_config{{id1, id2, id3}};
  pupperv3::ActuatorConfiguration actuator_config{{id1, id2, id3, id4, id5, id6}};

  // Create interface
  std::unique_ptr<pupperv3::MotorInterface> interface =
      std::make_unique<pupperv3::MotorInterface>(actuator_config);

  // ActuatorVector endstop_positions_degs = {-135, 90, 68, 135, -90, -68,
  //                                          -135, 90, 68, 135, -90, -68};
  ActuatorVector endstop_positions_degs = {-135, 90, 68, 135, -90, -68};
  // ActuatorVector endstop_positions_degs = {-135, 90, 68};
  ActuatorVector calibration_directions = {-1, 1, 1, 1, -1, -1};
  // ActuatorVector calibration_directions = {-1, 1, 1};

  // Create position controller
  pupperv3::MotorController<N_ACTUATORS> controller(
      /*position_kp (dps per output rad)=*/50000.0,
      /*speed_kp=*/5,
      /*max_speed (dps)=*/5000.0,
      /*endstop_positions_deg=*/endstop_positions_degs,
      /*calibration_directions=*/calibration_directions,
      /*motor_interface=*/std::move(interface));
  controller.begin();
  std::cout << "Initialized controller." << std::endl;

  controller.calibrate_motors(quit);

  // ActuatorVector command = {0, 0, 1.0, 0, 0, -1.0, 0, 0, 1.0, 0, 0, -1.0};
  ActuatorVector command = {0, 0, 1.0, 0, 0, -1.0};
  // ActuatorVector command = {0, 0, 1.0};
  controller.blocking_move(quit,
                           /*max_speed(rotor deg/s)=*/1000.0,
                           /*position_kp=*/20000.0,
                           /*speed_kp=*/10,
                           /*command=*/command,
                           /*speed_tolerance(output rad/s)=*/0.01,
                           /*wait_ticks=*/50);
  return 0;
}
