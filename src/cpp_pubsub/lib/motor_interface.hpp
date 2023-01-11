#pragma once

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/*
 * Current data structures:
 * e.g.
 * motor_connections_ = {CAN0, CAN1}
 * read_threads_ = {read CAN0, read CAN1}
 *
 * Should do:
 * Pass in either array with index being
 * motor ID in the robot and value being {CAN channel, CAN ID}
 * Then generate reverse map<can channel, map<can id, motor ID>>
 * This option makes more sense because less likely to mess it up.
 * Then can expose like position()->vector function, etc
 * The reading threads have to make map calls though to store data (~1ns per lookup?)
 */

static const int kNumCANChannels = 4;

enum class CANChannel {
  CAN0 = 0,
  CAN1 = 1,
  CAN2 = 2,
  CAN3 = 3,
};

const std::vector<CANChannel> kAllCANChannels = {CANChannel::CAN0, CANChannel::CAN1,
                                                 CANChannel::CAN2, CANChannel::CAN3};

struct CommonResponse {
  int16_t encoder_counts = 0;           // counts (-2^15-1 to 2^15)
  int16_t previous_encoder_counts = 0;  // previous counts
  float velocity_degs = 0.0;            // degs per sec
  float velocity_rads = 0.0;            // rads per sec
  float current = 0.0;                  // Amps
  uint8_t temp = 0;                     // C

  int32_t rotations = 0;            // motor rotations (post-processed)
  float multi_loop_angle = 0.0;     // degs (post-processed)
  float output_rads = 0.0;          // rads (post-processed)
  float output_rads_per_sec = 0.0;  // rads per sec
};

struct MultiLoopAngleResponse {
  float multi_loop_angle = 0.0;  // degs
};

struct MotorData {
  uint8_t error = 0;
  uint8_t motor_id = 0;
  MultiLoopAngleResponse multi_loop;
  CommonResponse common;
};

template <int ServosPerChannel>
class MotorInterface {
 public:
  using RobotMotorData = std::array<std::array<MotorData, ServosPerChannel>, kNumCANChannels>;

  explicit MotorInterface(std::vector<CANChannel> motor_connections);
  ~MotorInterface();
  void initialize_canbuses();
  void close_canbuses();
  void initialize_motors();
  void request_multi_angle(CANChannel bus, uint8_t motor_id);
  /* Command motor current in amps
   * Args:
   *   current: amps
   */
  void command_current(CANChannel bus, uint8_t motor_id, float current);
  /* Command motor velocity in deg/s
   * Args:
   *   velocity: deg/s
   */
  void command_velocity(CANChannel bus, uint8_t motor_id, float velocity);
  void write_pid_rom(CANChannel bus, uint8_t motor_id, uint8_t angle_kp, uint8_t angle_ki,
                     uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
  void write_pid_ram(CANChannel bus, uint8_t motor_id, uint8_t angle_kp, uint8_t angle_ki,
                     uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
  void write_pid_ram_to_all(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki,
                            uint8_t iq_kp, uint8_t iq_ki);
  void command_stop(CANChannel bus, uint8_t motor_id);
  void command_all_stop();
  void read_blocking(CANChannel bus);
  void start_read_threads();
  RobotMotorData motor_data_safe();

  /* Report the latest data from the motor
   * Multi_loop_angle: degs
   * velocity: deg/s
   * current: A
   */
  MotorData motor_data_safe(CANChannel bus, uint8_t motor_id);

  static const uint8_t kDefaultIqKp = 0x3C;
  static const uint8_t kDefaultIqKi = 0x28;

 private:
  void initialize_bus(CANChannel bus);
  void initialize_motor(CANChannel bus, uint8_t motor_id);
  struct can_frame read_canframe_blocking(CANChannel bus);
  uint32_t can_id(uint8_t motor_id);
  uint8_t get_motor_id(uint32_t can_id);
  void read_thread(CANChannel channel);
  void send(CANChannel bus, uint8_t motor_id, const std::array<uint8_t, 8> &payload);
  MotorData &motor_data(CANChannel bus, uint8_t motor_id);
  void parse_frame(CANChannel bus, const struct can_frame &frame);
  void multi_angle_update(CANChannel bus, uint8_t motor_id, const struct can_frame &frame);
  void torque_velocity_update(CANChannel bus, uint8_t motor_id, const struct can_frame &frame);
  void update_rotation(CommonResponse &common);
  std::string channel_str(CANChannel channel) {
    switch (channel) {
      case CANChannel::CAN0:
        return "can0";
      case CANChannel::CAN1:
        return "can2"; // TODO(nathankau) SUPER BAD NEED TO FIX
      case CANChannel::CAN2:
        return "can2";
      case CANChannel::CAN3:
        return "can3";
      default:
        std::cerr << "Invalid can channel" << std::endl;
        return "";
    }
  }
  std::array<int, 4> canbus_to_fd_;
  std::vector<CANChannel> motor_connections_;
  bool initialized_;
  std::atomic<bool> should_read_;

  const uint8_t kStartup0 = 0x76;
  const uint8_t kStartup1 = 0x88;
  const uint8_t kStartup2 = 0x77;
  const uint8_t kGetMultiAngle = 0x92;
  const uint8_t kCommandCurrent = 0xA1;
  const uint8_t kCommandVelocity = 0xA2;
  const uint8_t kCommandStop = 0x81;
  const uint8_t kWritePIDToROM = 0x32;
  const uint8_t kWritePIDToRAM = 0x31;
  const float kDegsPerTick = 0.01;
  const float kSpeedReduction = 0.1;

  const int kTimeoutSeconds = 1;

  // float latest_multi_angle_;
  RobotMotorData latest_data_;
  std::vector<std::shared_ptr<std::thread>> read_threads_;
  std::mutex latest_data_lock_;

  // DEBUG ONLY
  std::chrono::system_clock::time_point debug_start_;

  static constexpr float kCurrentWriteMax = 32.0;
  static const int kCurrentMultiplier = 2000;
  static const int kCurrentRawReadMax = 2048;
  static constexpr float kCurrentReadMax = 33.0;
  static const int kVelocityMultiplier = 100;
  static const int kEncoderCountsPerRot = 65536;
};
