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
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace pupperv3 {

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

enum class CANChannel {
  CAN0 = 0,
  CAN1 = 1,
  CAN2 = 2,
  CAN3 = 3,
};

inline std::string to_string(CANChannel channel) {
  switch (channel) {
    case CANChannel::CAN0:
      return "can0";
    case CANChannel::CAN1:
      return "can1";
    case CANChannel::CAN2:
      return "can2";
    case CANChannel::CAN3:
      return "can3";
    default:
      std::cerr << "Invalid can channel" << std::endl;
      return "";
  }
}

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

inline std::ostream& operator<<(std::ostream& os, const CommonResponse& common) {
  os << "common: ";
  os << "ec: " << common.encoder_counts << " ";
  os << "pec: " << common.previous_encoder_counts << " ";
  os << "vdeg: " << common.velocity_degs << " ";
  os << "vrad: " << common.velocity_rads << " ";
  os << "i: " << common.current << " ";
  os << "t: " << static_cast<int>(common.temp) << " ";
  os << "rots: " << common.rotations << " ";
  os << "mla: " << common.multi_loop_angle << " ";
  os << "orads: " << common.output_rads << " ";
  os << "oradspsec: " << common.output_rads_per_sec << " ";
  return os;
}

struct MultiLoopAngleResponse {
  float multi_loop_angle = 0.0;  // degs
};

inline std::ostream& operator<<(std::ostream& os, const MultiLoopAngleResponse& multi_loop) {
  os << "multi_loop: " << multi_loop.multi_loop_angle;
  return os;
}

struct MotorData {
  uint8_t error = 0;
  uint8_t motor_id = 0;
  MultiLoopAngleResponse multi_loop;
  CommonResponse common;
};

inline std::ostream& operator<<(std::ostream& os, const MotorData& md) {
  os << "motor data: "
     << "err: " << static_cast<int>(md.error) << " mid: " << static_cast<int>(md.motor_id) << " "
     << md.multi_loop << " " << md.common;
  return os;
}

struct MotorID {
  CANChannel bus;
  uint8_t motor_id;

  inline bool operator==(const MotorID& other) const {
    return other.bus == this->bus && other.motor_id == this->motor_id;
  }
};

using MotorIndex = int;
using ActuatorConfiguration = std::vector<MotorID>;

class MotorInterface {
 public:
  // Specify motor configuration like so: [(canbus, id),(canbus, id)...]
  // Build inverse map automatically: map<(canbus, id), index>

  // Since hashing MotorID requires boost, use vector::find every time to find index
  // using ActuatorConfigurationInverse = std::map<MotorID, MotorIndex>;

  using ActuatorData = std::vector<MotorData>;

  explicit MotorInterface(ActuatorConfiguration actuator_config, bool verbose = false);
  ~MotorInterface();
  void initialize_canbuses();
  void close_canbuses();
  void initialize_motors();
  void request_multi_angle(const MotorID& motor_id);
  /* Command motor current in amps
   * Args:
   *   current: amps
   */
  void command_current(const MotorID& motor_id, float current);
  /* Command motor velocity in deg/s
   * Args:
   *   velocity: deg/s
   */
  void command_velocity(const MotorID& motor_id, float velocity);
  void write_pid_rom(const MotorID& motor_id, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                     uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
  void write_pid_ram(const MotorID& motor_id, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                     uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
  void write_pid_ram_to_all(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki,
                            uint8_t iq_kp, uint8_t iq_ki);
  void command_stop(const MotorID& motor_id);
  void command_all_stop();
  void read_blocking(CANChannel bus);
  void start_read_threads();
  ActuatorData motor_data_safe();

  /* Report the latest data from the motor
   * Multi_loop_angle: degs
   * velocity: deg/s
   * current: A
   */
  MotorData motor_data_safe(const MotorID& motor_id);

  static const uint8_t kDefaultIqKp = 0x3C;
  static const uint8_t kDefaultIqKi = 0x28;

 private:
  void initialize_bus(CANChannel bus);
  void initialize_motor(const MotorID& motor_id);
  struct can_frame read_canframe_blocking(CANChannel bus);
  static uint32_t motor_id_to_can_id(uint8_t motor_id);
  static uint8_t can_id_to_motor_id(uint32_t can_id);
  void read_thread(CANChannel channel);
  void send(const MotorID& motor_id, const std::array<uint8_t, 8>& payload);

  // O(N) search for motor given its ID and then return data
  // O(N) search is probably faster than hashmap at the typical number of motors (12)
  MotorData& motor_data(MotorID motor_id);
  void parse_frame(CANChannel bus, const struct can_frame& frame);
  void multi_angle_update(const MotorID& motor_id, const struct can_frame& frame);
  void torque_velocity_update(const MotorID& motor_id, const struct can_frame& frame);
  void update_rotation(CommonResponse& common);

  std::unordered_map<CANChannel, int> canbus_to_fd_;
  ActuatorConfiguration actuator_config_;
  bool initialized_;
  std::atomic<bool> should_read_;

  static constexpr uint8_t kStartup0 = 0x76;
  static constexpr uint8_t kStartup1 = 0x88;
  static constexpr uint8_t kStartup2 = 0x77;
  static constexpr uint8_t kGetMultiAngle = 0x92;
  static constexpr uint8_t kCommandCurrent = 0xA1;
  static constexpr uint8_t kCommandVelocity = 0xA2;
  static constexpr uint8_t kCommandStop = 0x81;
  static constexpr uint8_t kWritePIDToROM = 0x32;
  static constexpr uint8_t kWritePIDToRAM = 0x31;
  static constexpr float kDegsPerTick = 0.01;
  static constexpr float kSpeedReduction = 0.1;

  static constexpr int kTimeoutSeconds = 1;

  ActuatorData latest_data_;
  std::mutex latest_data_lock_;

  std::unordered_set<CANChannel> canbuses_;
  std::unordered_map<CANChannel, std::shared_ptr<std::thread>> read_threads_;

  // DEBUG ONLY
  std::chrono::system_clock::time_point debug_start_;

  bool verbose_;

  static constexpr float kCurrentWriteMax = 32.0;
  static constexpr int kCurrentMultiplier = 2000;
  static constexpr int kCurrentRawReadMax = 2048;
  static constexpr float kCurrentReadMax = 33.0;
  static constexpr int kVelocityMultiplier = 100;
  static constexpr int kEncoderCountsPerRot = 65536;
};

inline std::ostream& operator<<(std::ostream& os, const MotorInterface::ActuatorData& at) {
  for (const auto& datum : at) {
    os << datum << "\n";
  }
  os << "\n";
  return os;
}

}  // namespace pupperv3