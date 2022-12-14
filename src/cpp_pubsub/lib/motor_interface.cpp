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

#include "motor_interface.hpp"
#include "prof_utils.hpp"

#define TEMPLATE_HEADER template <int kServosPerChannel>
#define MOTOR_INTERFACE MotorInterface<kServosPerChannel>

#define DEG_TO_RAD 0.01745329252

TEMPLATE_HEADER
MOTOR_INTERFACE::MotorInterface(std::vector<CANChannel> motor_connections)
    : motor_connections_(motor_connections), initialized_(false), should_read_(true) {
  // DEBUG ONLY
  debug_start_ = time_now();
  canbus_to_fd_.fill(-1);
  for (size_t bus_idx = 0; bus_idx < latest_data_.size(); bus_idx++) {
    for (size_t motor_idx = 0; motor_idx < latest_data_.at(0).size(); motor_idx++) {
      latest_data_.at(bus_idx).at(motor_idx) = MotorData{};
    }
  }
}

TEMPLATE_HEADER
MOTOR_INTERFACE::~MotorInterface() {
  std::cout << "Calling motor interface destructor." << std::endl;
  std::cout << "Stopping all motors." << std::endl;
  command_all_stop();
  std::cout << "Signaling read threads to stop." << std::endl;
  should_read_.store(false);
  //   Causes lots of can socket exceptions / errors
  for (size_t i = 0; i < read_threads_.size(); i++) {
    auto active_thread = read_threads_.at(i);
    if (active_thread) {
      std::cout << "Joining read thread for " << channel_str(motor_connections_.at(i)) << ". ";
      active_thread->join();
    }
  }

  close_canbuses();
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_canbuses() {
  for (const auto &bus : motor_connections_) {
    initialize_bus(bus);
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::close_canbuses() {
  std::cout << "\nClosing can bus sockets...";
  for (const auto &bus : motor_connections_) {
    close(canbus_to_fd_.at(static_cast<int>(bus)));
  }
  std::cout << "closed." << std::endl;
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_motors() {
  for (const auto &bus : motor_connections_) {
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
      std::cout << "Initializing motor id: " << motor_id;
      std::cout << " on channel: " << static_cast<int>(bus) << std::endl;
      initialize_motor(bus, motor_id);
    }
  }
  initialized_ = true;
}

TEMPLATE_HEADER
typename MOTOR_INTERFACE::RobotMotorData MOTOR_INTERFACE::motor_data_safe() {
  {
    std::unique_lock<std::mutex> lock(latest_data_lock_);
    return latest_data_;
  }
}

/*
Amps
*/
TEMPLATE_HEADER
void MOTOR_INTERFACE::command_current(CANChannel bus, uint8_t motor_id, float current) {
  int16_t discrete_current = static_cast<int16_t>(current / kCurrentWriteMax * kCurrentMultiplier);
  uint8_t lsb = discrete_current & 0xFF;
  uint8_t gsb = (discrete_current >> 8) & 0xFF;
  send(bus, motor_id, {kCommandCurrent, 0, 0, 0, lsb, gsb, 0, 0});
}

/*
DEG/S
*/
TEMPLATE_HEADER
void MOTOR_INTERFACE::command_velocity(CANChannel bus, uint8_t motor_id, float velocity) {
  int32_t discrete_velocity = static_cast<int32_t>(velocity * kVelocityMultiplier);
  uint8_t lsb = discrete_velocity & 0xFF;
  uint8_t byte1 = (discrete_velocity >> 8) & 0xFF;
  uint8_t byte2 = (discrete_velocity >> 16) & 0xFF;
  uint8_t byte3 = (discrete_velocity >> 24) & 0xFF;
  send(bus, motor_id, {kCommandVelocity, 0, 0, 0, lsb, byte1, byte2, byte3});
}

std::array<uint8_t, 8> pid_message(uint8_t command, uint8_t angle_kp, uint8_t angle_ki,
                                   uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
                                   uint8_t iq_ki) {
  return {command, 0, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki};
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::write_pid_rom(CANChannel bus, uint8_t motor_id, uint8_t angle_kp,
                                    uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki,
                                    uint8_t iq_kp, uint8_t iq_ki) {
  send(bus, motor_id,
       pid_message(kWritePIDToROM, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki));
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::write_pid_ram(CANChannel bus, uint8_t motor_id, uint8_t angle_kp,
                                    uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki,
                                    uint8_t iq_kp, uint8_t iq_ki) {
  send(bus, motor_id,
       pid_message(kWritePIDToRAM, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki));
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::write_pid_ram_to_all(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                                           uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki) {
  for (auto bus : motor_connections_) {
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
      write_pid_ram(bus, motor_id, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
    }
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::command_stop(CANChannel bus, uint8_t motor_id) {
  send(bus, motor_id, {kCommandStop, 0, 0, 0, 0, 0, 0, 0});
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::command_all_stop() {
  for (auto bus : motor_connections_) {
    for (int motor_id = 1; motor_id <= kServosPerChannel; motor_id++) {
      command_stop(bus, motor_id);
    }
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::request_multi_angle(CANChannel bus, uint8_t motor_id) {
  auto start = time_now();
  send(bus, motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
  auto stop = time_now();
  // std::cout << "Send (ns): " << duration_ns(stop - start) << "\t"; // Send takes roughly 81us
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::update_rotation(CommonResponse &common) {
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

TEMPLATE_HEADER
void MOTOR_INTERFACE::read_blocking(CANChannel bus) {
  struct can_frame frame = read_canframe_blocking(bus);
  std::cout << "Read Bus: " << channel_str(bus) << " CAN ID: " << frame.can_id << std::endl;
  parse_frame(bus, frame);
}

TEMPLATE_HEADER
struct can_frame MOTOR_INTERFACE::read_canframe_blocking(CANChannel bus) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  // Print time to get fd
  auto start = time_now();
  int file_descriptor = canbus_to_fd_.at(static_cast<int>(bus));
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

TEMPLATE_HEADER
MotorData &MOTOR_INTERFACE::motor_data(CANChannel bus, uint8_t motor_id) {
  return latest_data_.at(static_cast<int>(bus)).at(motor_id - 1);
}

TEMPLATE_HEADER
MotorData MOTOR_INTERFACE::motor_data_safe(CANChannel bus, uint8_t motor_id) {
  std::unique_lock<std::mutex> lock(latest_data_lock_);
  return latest_data_.at(static_cast<int>(bus)).at(motor_id - 1);
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::multi_angle_update(CANChannel bus, uint8_t motor_id,
                                         const struct can_frame &frame) {
  int64_t multi_loop_angle = 0;
  memcpy(&multi_loop_angle, frame.data, 8);
  multi_loop_angle = multi_loop_angle >> 8;
  {
    std::unique_lock<std::mutex> lock(latest_data_lock_);
    motor_data(bus, motor_id).multi_loop.multi_loop_angle =
        static_cast<float>(multi_loop_angle) * kDegsPerTick * kSpeedReduction;
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::torque_velocity_update(CANChannel bus, uint8_t motor_id,
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
    CommonResponse &common = motor_data(bus, motor_id).common;
    common.temp = temp_raw;
    common.current = static_cast<float>(current_raw) * kCurrentReadMax / kCurrentRawReadMax;
    common.velocity_degs = speed_raw;
    common.velocity_rads = common.velocity_degs * DEG_TO_RAD;
    common.output_rads_per_sec = common.velocity_rads * kSpeedReduction;
    common.encoder_counts = encoder_counts;
    update_rotation(common);
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::parse_frame(CANChannel bus, const struct can_frame &frame) {
  uint8_t motor_id = get_motor_id(frame.can_id);
  if (motor_id <= 0 || motor_id > kServosPerChannel) {
    std::cout << "Invalid motor id." << std::endl;
    return;
  }

  uint8_t command_id = frame.data[0];
  if (command_id == kGetMultiAngle) {
    multi_angle_update(bus, motor_id, frame);
    std::cout << "got multi angle" << std::endl;
  }
  if (command_id == kCommandCurrent || command_id == kCommandVelocity) {
    torque_velocity_update(bus, motor_id, frame);
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::start_read_threads() {
  for (auto canbus : motor_connections_) {
    std::cout << "Starting read thread on bus: " << channel_str(canbus) << std::endl;
    read_threads_.push_back(
        std::make_shared<std::thread>(&MOTOR_INTERFACE::read_thread, this, canbus));
  }
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_bus(CANChannel bus) {
  int socket_id;
  struct sockaddr_can addr;
  struct ifreq ifr;
  std::cout << "Initializing " << channel_str(bus) << "." << std::endl;
  if ((socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cerr << "socket\n";
  }

  // find interface index
  strcpy(ifr.ifr_name, channel_str(bus).c_str());
  ifr.ifr_ifindex = static_cast<int>(if_nametoindex(ifr.ifr_name));
  if (ifr.ifr_ifindex == 0) {
    std::cerr << "if_nametoindex\n";
  }
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  std::cout << "Found: " << channel_str(bus) << " ifindex: " << addr.can_ifindex << std::endl;

  // bind socket
  if (bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cerr << "bind\n";
  }
  canbus_to_fd_.at(static_cast<int>(bus)) = socket_id;

  // Set timeout
  struct timeval tv;
  tv.tv_sec = kTimeoutSeconds;
  tv.tv_usec = 0;
  setsockopt(socket_id, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
}

TEMPLATE_HEADER
void MOTOR_INTERFACE::initialize_motor(CANChannel bus, uint8_t motor_id) {
  send(bus, motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
  send(bus, motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
  send(bus, motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
  usleep(10000);
}

TEMPLATE_HEADER
uint32_t MOTOR_INTERFACE::can_id(uint8_t motor_id) { return 0x140 + motor_id; }

TEMPLATE_HEADER
uint8_t MOTOR_INTERFACE::get_motor_id(uint32_t can_id) { return can_id - 0x140; }

TEMPLATE_HEADER
void MOTOR_INTERFACE::read_thread(CANChannel channel) {
  while (should_read_.load()) {
    read_blocking(channel);
  }
}

/* Only non-thread-safe part of this function is reading canbus_to_fd_.
 * However, it's array of int so probably ok */
TEMPLATE_HEADER
void MOTOR_INTERFACE::send(CANChannel bus, uint8_t motor_id,
                           const std::array<uint8_t, 8> &payload) {
  // std::cout << "Send can message\n";
  int file_descriptor = canbus_to_fd_.at(static_cast<int>(bus));
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id(motor_id);
  frame.len = 8;
  memcpy(frame.data, payload.data(), 8);
  auto start = time_now();
  if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU) {
    std::cerr << "Error writing frame to " << channel_str(bus) << "\n";
  }
  auto stop = time_now();
  // std::cout << "Write (ns): " << duration_ns(stop - start) << "\t"; // Takes around 80us
  // std::cout << "Send start (ms): " << duration_ms(start - debug_start_) << "\t";
  // std::cout << "Send end (ms): " << duration_ms(stop - debug_start_) << "\t";
}

template class MotorInterface<1>;
template class MotorInterface<2>;
template class MotorInterface<3>;
template class MotorInterface<4>;
template class MotorInterface<5>;
template class MotorInterface<6>;
