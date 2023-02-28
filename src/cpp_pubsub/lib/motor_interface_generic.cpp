#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "motor_interface_generic.hpp"
#include "prof_utils.hpp"

namespace pupperv3 {

#define DEG_TO_RAD 0.01745329252

MotorInterface::MotorInterface(ActuatorConfiguration actuator_config, bool verbose)
    : actuator_config_(actuator_config),
      initialized_(false),
      should_read_(true),
      verbose_(verbose) {
  // DEBUG ONLY
  debug_start_ = time_now();
  for (auto const &motor_id : actuator_config) {
    latest_data_.emplace_back();
    canbuses_.insert(motor_id.bus);
  }
}

MotorInterface::~MotorInterface() {
  std::cout << "Calling motor interface destructor.\n";
  std::cout << "Stopping all motors.\n";
  command_all_stop();

  std::cout << "Signaling read threads to stop.\n";
  should_read_.store(false);

  //   Causes lots of can socket exceptions / errors
  for (auto &[bus, thread] : read_threads_) {
    if (thread) {
      std::cout << "Joining read thread for " << to_string(bus) << ". ";
      thread->join();
    }
  }

  close_canbuses();
}

void MotorInterface::initialize_canbuses() {
  for (const auto &bus : canbuses_) {
    initialize_bus(bus);
  }
}

void MotorInterface::close_canbuses() {
  std::cout << "\nClosing can bus sockets...";
  for (const auto &bus : canbuses_) {
    close(canbus_to_fd_.at(bus));
  }
  std::cout << "closed." << std::endl;
}

void MotorInterface::initialize_motors() {
  for (const auto &motor_id : actuator_config_) {
    initialize_motor(motor_id);
  }
  initialized_ = true;
}

MotorInterface::ActuatorData MotorInterface::motor_data_safe() {
  {
    std::unique_lock<std::mutex> lock(latest_data_lock_);
    return latest_data_;
  }
}

/*
Amps
*/
void MotorInterface::command_current(const MotorID &motor_id, float current) {
  int16_t discrete_current = static_cast<int16_t>(current / kCurrentWriteMax * kCurrentMultiplier);
  uint8_t lsb = discrete_current & 0xFF;
  uint8_t gsb = (discrete_current >> 8) & 0xFF;
  send(motor_id, {kCommandCurrent, 0, 0, 0, lsb, gsb, 0, 0});
}

/*
DEG/S
*/
void MotorInterface::command_velocity(const MotorID &motor_id, float velocity) {
  int32_t discrete_velocity = static_cast<int32_t>(velocity * kVelocityMultiplier);
  uint8_t lsb = discrete_velocity & 0xFF;
  uint8_t byte1 = (discrete_velocity >> 8) & 0xFF;
  uint8_t byte2 = (discrete_velocity >> 16) & 0xFF;
  uint8_t byte3 = (discrete_velocity >> 24) & 0xFF;
  send(motor_id, {kCommandVelocity, 0, 0, 0, lsb, byte1, byte2, byte3});
}

std::array<uint8_t, 8> pid_message(uint8_t command, uint8_t angle_kp, uint8_t angle_ki,
                                   uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
                                   uint8_t iq_ki) {
  return {command, 0, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki};
}

void MotorInterface::write_pid_rom(const MotorID &motor_id, uint8_t angle_kp, uint8_t angle_ki,
                                   uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
                                   uint8_t iq_ki) {
  send(motor_id, pid_message(kWritePIDToROM, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki));
}

void MotorInterface::write_pid_ram(const MotorID &motor_id, uint8_t angle_kp, uint8_t angle_ki,
                                   uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
                                   uint8_t iq_ki) {
  send(motor_id, pid_message(kWritePIDToRAM, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki));
}

void MotorInterface::write_pid_ram_to_all(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                                          uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki) {
  for (const auto &motor_id : actuator_config_) {
    write_pid_ram(motor_id, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
  }
}

void MotorInterface::command_stop(const MotorID &motor_id) {
  send(motor_id, {kCommandStop, 0, 0, 0, 0, 0, 0, 0});
}

void MotorInterface::command_all_stop() {
  for (const auto &motor_id : actuator_config_) {
    command_stop(motor_id);
  }
}

void MotorInterface::request_multi_angle(const MotorID &motor_id) {
  auto start = time_now();
  send(motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
  auto stop = time_now();
  // std::cout << "Send (ns): " << duration_ns(stop - start) << "\t"; // Send takes roughly 81us
}

void MotorInterface::update_rotation(CommonResponse &common) {
  if (common.encoder_counts - common.previous_encoder_counts > kEncoderCountsPerRot / 2) {
    common.rotations--;
  }
  if (common.encoder_counts - common.previous_encoder_counts < -kEncoderCountsPerRot / 2) {
    common.rotations++;
  }
  common.previous_encoder_counts = common.encoder_counts;
  common.multi_loop_angle = (static_cast<float>(common.rotations) +
                             static_cast<float>(common.encoder_counts) / kEncoderCountsPerRot) *
                            360.0F;
  common.output_rads = common.multi_loop_angle * kSpeedReduction * DEG_TO_RAD;
}

void MotorInterface::read_blocking(CANChannel bus) {
  struct can_frame frame = read_canframe_blocking(bus);
  if (verbose_) {
    std::cout << "Read Bus: " << to_string(bus) << " CAN ID: " << frame.can_id << "\n";
  }
  parse_frame(bus, frame);
}

struct can_frame MotorInterface::read_canframe_blocking(CANChannel bus) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  // Print time to get fd
  auto start = time_now();
  int file_descriptor = canbus_to_fd_.at(bus);
  auto stop = time_now();
  // std::cout << "FD lookup (ns): " << duration_ns(stop - start) << "\t";

  // Print time to read from can bus
  auto start_read = time_now();
  long nbytes = read(file_descriptor, &frame, sizeof(struct can_frame));
  auto stop_read = time_now();
  // std::cout << "Read start (ms): " << duration_ms(start_read - debug_start_) << "\t";
  // std::cout << "Read end (ms): " << duration_ms(stop_read - debug_start_) << "\t";
  if (nbytes < 0) {
    // Continue on read timeouts
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      std::cout << "Bus " << static_cast<int>(bus) << " timed out on read" << std::endl;
    }
    std::cerr << "ERROR: can raw socket read\n";
  }
  if (nbytes != sizeof(struct can_frame)) {
    std::cerr << "ERROR: did not read full can frame\n";
  }
  stop = time_now();
  // std::cout << "CAN read (ns): " << duration_ns(stop_read - start_read) << "\t";
  return frame;
}

MotorData &MotorInterface::motor_data(MotorID motor_id) {
  for (int i = 0; i < actuator_config_.size(); i++) {
    if (actuator_config_[i] == motor_id) {
      return latest_data_.at(i);
    }
  }
  throw std::invalid_argument("Could not find actuator with given motor_id");
}

MotorData MotorInterface::motor_data_safe(const MotorID &motor_id) {
  std::unique_lock<std::mutex> lock(latest_data_lock_);
  return motor_data(motor_id);
}

void MotorInterface::multi_angle_update(const MotorID &motor_id, const struct can_frame &frame) {
  int64_t multi_loop_angle = 0;
  memcpy(&multi_loop_angle, frame.data, 8);
  multi_loop_angle = multi_loop_angle >> 8;
  {
    std::unique_lock<std::mutex> lock(latest_data_lock_);
    motor_data(motor_id).multi_loop.multi_loop_angle =
        static_cast<float>(multi_loop_angle) * kDegsPerTick * kSpeedReduction;
  }
}

void MotorInterface::torque_velocity_update(const MotorID &motor_id,
                                            const struct can_frame &frame) {
  int8_t temp_raw = static_cast<int8_t>(frame.data[1]);
  int16_t current_raw;
  memcpy(&current_raw, frame.data + 2, 2);
  int16_t speed_raw;
  memcpy(&speed_raw, frame.data + 4, 2);
  int16_t encoder_counts;
  memcpy(&encoder_counts, frame.data + 6, 2);

  // TODO: Implement per-motor mutex
  {
    std::unique_lock<std::mutex> lock(latest_data_lock_);
    auto &data = motor_data(motor_id);
    data.motor_id = motor_id.motor_id;
    data.common.temp = temp_raw;
    data.common.current = static_cast<float>(current_raw) * kCurrentReadMax / kCurrentRawReadMax;
    data.common.velocity_degs = speed_raw;
    data.common.velocity_rads = data.common.velocity_degs * DEG_TO_RAD;
    data.common.output_rads_per_sec = data.common.velocity_rads * kSpeedReduction;
    data.common.encoder_counts = encoder_counts;
    update_rotation(data.common);
  }
}

void MotorInterface::parse_frame(CANChannel bus, const struct can_frame &frame) {
  MotorID motor_id;
  motor_id.bus = bus;
  motor_id.motor_id = can_id_to_motor_id(frame.can_id);
  if (motor_id.motor_id <= 0) {
    std::cout << "Invalid motor id: " << motor_id.motor_id << '\n';
    return;
  }

  uint8_t command_id = frame.data[0];
  if (command_id == kGetMultiAngle) {
    multi_angle_update(motor_id, frame);
    std::cout << "got multi angle" << std::endl;
  }
  if (command_id == kCommandCurrent || command_id == kCommandVelocity) {
    torque_velocity_update(motor_id, frame);
  }
}

void MotorInterface::start_read_threads() {
  for (auto canbus : canbuses_) {
    std::cout << "Starting read thread on bus: " << to_string(canbus) << std::endl;
    read_threads_.insert(
        {canbus, std::make_shared<std::thread>(&MotorInterface::read_thread, this, canbus)});
  }
}

void MotorInterface::initialize_bus(CANChannel bus) {
  int socket_id;
  struct sockaddr_can addr;
  struct ifreq ifr;
  std::cout << "Initializing " << to_string(bus) << "." << std::endl;
  if ((socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cerr << "socket\n";
  }

  // find interface index
  strcpy(ifr.ifr_name, to_string(bus).c_str());
  ifr.ifr_ifindex = static_cast<int>(if_nametoindex(ifr.ifr_name));
  if (ifr.ifr_ifindex == 0) {
    std::cerr << "if_nametoindex\n";
  }
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  std::cout << "Found: " << to_string(bus) << " ifindex: " << addr.can_ifindex << std::endl;

  // bind socket
  if (bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cerr << "bind\n";
  }
  canbus_to_fd_.insert({bus, socket_id});

  // Set timeout
  struct timeval tv;
  tv.tv_sec = kTimeoutSeconds;
  tv.tv_usec = 0;
  setsockopt(socket_id, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
}

void MotorInterface::initialize_motor(const MotorID &motor_id) {
  std::cout << "Initializing motor id: " << static_cast<int>(motor_id.motor_id);
  std::cout << " on channel: " << to_string(motor_id.bus) << std::endl;

  send(motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
  send(motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
  send(motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
}

uint32_t MotorInterface::motor_id_to_can_id(uint8_t motor_id) { return 0x140 + motor_id; }

uint8_t MotorInterface::can_id_to_motor_id(uint32_t can_id) { return can_id - 0x140; }

void MotorInterface::read_thread(CANChannel channel) {
  while (should_read_.load()) {
    read_blocking(channel);
  }
}

/* Only non-thread-safe part of this function is reading canbus_to_fd_.
 * However, it's array of int so probably ok */
void MotorInterface::send(const MotorID &motor_id, const std::array<uint8_t, 8> &payload) {
  // std::cout << "Send can message\n";
  int file_descriptor = canbus_to_fd_.at(motor_id.bus);
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = motor_id_to_can_id(motor_id.motor_id);
  frame.len = 8;
  memcpy(frame.data, payload.data(), 8);
  auto start = time_now();
  if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU) {
    std::cerr << "Error writing frame to " << to_string(motor_id.bus) << "\n";
  }
  auto stop = time_now();
  // std::cout << "Write (ns): " << duration_ns(stop - start) << "\t"; // Takes around 80us
  // std::cout << "Send start (ms): " << duration_ms(start - debug_start_) << "\t";
  // std::cout << "Send end (ms): " << duration_ms(stop - debug_start_) << "\t";
}

}  // namespace pupperv3