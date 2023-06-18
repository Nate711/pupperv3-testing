#include "motor_interface_generic.hpp"
#include "prof_utils.hpp"

// May need to define this idk
// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

namespace pupperv3 {

#define DEG_TO_RAD 0.01745329252

MotorInterface::MotorInterface(ActuatorConfiguration actuator_config)
    : MotorInterface(actuator_config, false) {}

MotorInterface::MotorInterface(ActuatorConfiguration actuator_config, bool verbose)
    : actuator_config_(actuator_config),
      initialized_(false),
      should_read_(true),
      verbose_(verbose) {
  // DEBUG ONLY
  debug_start_ = prof_utils::now();
  for (auto const &motor_id : actuator_config) {
    latest_data_.emplace_back();

    // initialize debug arrays
    messages_sent_.push_back(std::make_shared<std::atomic_int>(0));
    messages_received_.push_back(std::make_shared<std::atomic_int>(0));
    messages_sent_since_last_receive_.push_back(std::make_shared<std::atomic_int>(0));
    time_last_sent_.emplace_back();
    time_last_received_.push_back(
        std::make_shared<std::atomic<std::chrono::time_point<std::chrono::high_resolution_clock>>>(
            prof_utils::now()));

    canbuses_.insert(motor_id.bus);
  }
  initialize_canbuses();
}

MotorInterface::~MotorInterface() {
  SPDLOG_INFO("Destroying motor interface...");
  SPDLOG_INFO("Stopping all motors.");
  command_all_stop();

  SPDLOG_INFO("Signaling read threads to stop...");
  should_read_.store(false);

  //   Causes lots of can socket exceptions / errors
  for (auto &[bus, thread] : read_threads_) {
    if (thread) {
      SPDLOG_INFO("Joining read thread for {}", to_string(bus));
      thread->join();
    }
  }

  close_canbuses();
  SPDLOG_INFO("Destroying motor interface members...");
}

void MotorInterface::initialize_canbuses() {
  for (const auto &bus : canbuses_) {
    initialize_bus(bus);
  }
}

void MotorInterface::close_canbuses() {
  SPDLOG_INFO("Closing CAN bus sockets...");
  for (const auto &bus : canbuses_) {
    close(canbus_to_filedescriptor_.at(bus));
  }
  SPDLOG_INFO("Closed all CAN bus sockets.");
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
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(500us);
  for (const auto &motor_id : actuator_config_) {
    write_pid_ram(motor_id, angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
    std::this_thread::sleep_for(500us);
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
  auto start = prof_utils::now();
  send(motor_id, {kGetMultiAngle, 0, 0, 0, 0, 0, 0, 0});
  auto stop = prof_utils::now();
  // SPDLOG_INFO("Send (ns): " <<prof_utils::duration_ns(stop - start) << "\t"; // Send takes
  // roughly 81us
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
  auto maybe_frame = read_canframe_blocking(bus);
  if (!maybe_frame) {
    SPDLOG_ERROR("Could not read CAN frame");
    return;
  }
  if (verbose_) {
    SPDLOG_INFO("Read @ {}:{}", to_string(bus), maybe_frame->can_id);
  }
  parse_frame(bus, *maybe_frame);
}

std::optional<struct can_frame> MotorInterface::read_canframe_blocking(CANChannel bus) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  // Print time to get file descriptor
  // auto start = prof_utils::now();
  int file_descriptor = canbus_to_filedescriptor_.at(bus);
  // auto stop = prof_utils::now();
  // SPDLOG_INFO("File descriptor lookup (ns): " <<prof_utils::duration_ns(stop - start) << "\t";

  // Print time to read from can bus
  // auto start_read = prof_utils::now();
  long nbytes = read(file_descriptor, &frame, sizeof(struct can_frame));
  // auto stop_read = prof_utils::now();
  // SPDLOG_INFO("Read start (ms): " <<prof_utils::duration_ms(start_read - debug_start_) << "\t";
  // SPDLOG_INFO("Read end (ms): " <<prof_utils::duration_ms(stop_read - debug_start_) << "\t";
  if (nbytes < 0) {
    // Continue on read timeouts
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      SPDLOG_INFO("Bus {} timed out on read", to_string(bus));
    }
    SPDLOG_ERROR("ERROR: can raw socket read");
  }
  if (nbytes != sizeof(struct can_frame)) {
    SPDLOG_ERROR("ERROR: did not read full can frame ({} of {})", nbytes, sizeof(struct can_frame));
    return std::nullopt;
  }
  // stop = prof_utils::now();
  // SPDLOG_INFO("CAN read (ns): " <<prof_utils::duration_ns(stop_read - start_read) << "\t";
  return frame;
}

int MotorInterface::motor_flat_index(MotorID motor_id) const {
  for (int i = 0; i < actuator_config_.size(); i++) {
    if (actuator_config_[i] == motor_id) {
      return i;
    }
  }
  SPDLOG_ERROR("motor_id not found: {}", motor_id);
  throw std::invalid_argument("Could not find actuator with given motor_id");
}

MotorData &MotorInterface::motor_data(MotorID motor_id) {
  return latest_data_[motor_flat_index(motor_id)];
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

  int motor_idx = motor_flat_index(motor_id);
  (*messages_received_.at(motor_idx))++;
  if (verbose_) {
    SPDLOG_INFO("- Bus: {} motor_id: {}", to_string(bus), motor_id.motor_id);
    SPDLOG_INFO("- Msgs sent before this receive: {}",
                messages_sent_since_last_receive_.at(motor_idx)->load());
  }
  (*messages_sent_since_last_receive_.at(motor_idx)) = 0;
  *time_last_received_.at(motor_idx) = prof_utils::now();
  if (verbose_) {
    SPDLOG_INFO("- Time since sent: {} ns",
                prof_utils::duration_ns(prof_utils::now() - time_last_sent_.at(motor_idx)));
  }

  if (motor_id.motor_id <= 0) {
    SPDLOG_INFO("Invalid motor id: {}", motor_id.motor_id);
    return;
  }

  uint8_t command_id = frame.data[0];
  if (command_id == kGetMultiAngle) {
    multi_angle_update(motor_id, frame);
    SPDLOG_INFO("got multi angle");
  }
  if (command_id == kCommandCurrent || command_id == kCommandVelocity) {
    torque_velocity_update(motor_id, frame);
  }
}

void MotorInterface::start_read_threads() {
  for (auto canbus : canbuses_) {
    SPDLOG_INFO("Starting read thread on bus: {}", to_string(canbus));
    read_threads_.insert(
        {canbus, std::make_shared<std::thread>(&MotorInterface::read_thread, this, canbus)});
  }
}

void MotorInterface::initialize_bus(CANChannel bus) {
  int socket_id;
  struct sockaddr_can addr;
  struct ifreq ifr;
  SPDLOG_INFO("Initializing {}", to_string(bus));
  if ((socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    SPDLOG_ERROR("socket");
    throw std::runtime_error("could not construct can socket object");
  }

  // find interface index
  strcpy(ifr.ifr_name, to_string(bus).c_str());
  ifr.ifr_ifindex = static_cast<int>(if_nametoindex(ifr.ifr_name));
  if (ifr.ifr_ifindex == 0) {
    SPDLOG_ERROR("could not find can bus named: {}", to_string(bus));
    throw std::invalid_argument("could not find can bus");
  }
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  SPDLOG_INFO("Found: {} ifindex: {}", to_string(bus), addr.can_ifindex);

  // bind socket
  if (bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    SPDLOG_ERROR("bind");
    throw std::runtime_error("could nto bind");
  }
  canbus_to_filedescriptor_.insert({bus, socket_id});

  // Set timeout
  struct timeval tv;
  tv.tv_sec = kTimeoutSeconds;
  tv.tv_usec = kTimeoutMicroseconds;
  setsockopt(socket_id, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
}

void MotorInterface::initialize_motor(const MotorID &motor_id) {
  SPDLOG_INFO("Initializing motor id: {} on bus: {}", static_cast<int>(motor_id.motor_id),
              to_string(motor_id.bus));
  using namespace std::chrono_literals;
  send(motor_id, {kStartup0, 0, 0, 0, 0, 0, 0, 0});
  std::this_thread::sleep_for(1ms);
  send(motor_id, {kStartup1, 0, 0, 0, 0, 0, 0, 0});
  std::this_thread::sleep_for(1ms);
  send(motor_id, {kStartup2, 0, 0, 0, 0, 0, 0, 0});
  std::this_thread::sleep_for(1ms);
}

uint32_t MotorInterface::motor_id_to_can_id(uint8_t motor_id) { return 0x140 + motor_id; }

uint8_t MotorInterface::can_id_to_motor_id(uint32_t can_id) { return can_id - 0x140; }

void MotorInterface::read_thread(CANChannel channel) {
  while (should_read_.load()) {
    read_blocking(channel);
  }
}

/* Only non-thread-safe part of this function is reading canbus_to_filedescriptor_ which is
 * protected with lock
 */
void MotorInterface::send(const MotorID &motor_id, const std::array<uint8_t, 8> &payload) {
  // SPDLOG_INFO("Send can message\n";
  int file_descriptor = -1;
  {
    std::unique_lock<std::mutex> lock(canbus_to_filedescriptor_lock_);
    file_descriptor = canbus_to_filedescriptor_.at(motor_id.bus);
  }
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = motor_id_to_can_id(motor_id.motor_id);
  frame.len = 8;
  memcpy(frame.data, payload.data(), 8);
  auto start = prof_utils::now();
  if (write(file_descriptor, &frame, CAN_MTU) != CAN_MTU) {
    SPDLOG_ERROR("Error writing frame to {}", to_string(motor_id.bus));
  }
  auto stop = prof_utils::now();

  // Record that we sent a message to this motor
  time_last_sent_[motor_flat_index(motor_id)] = prof_utils::now();
  (*messages_sent_.at(motor_flat_index(motor_id)))++;
  (*messages_sent_since_last_receive_.at(motor_flat_index(motor_id)))++;

  if (verbose_) {
    SPDLOG_INFO("Send @ {}:{}", to_string(motor_id.bus), motor_id.motor_id);
    SPDLOG_INFO("- sent: {} received: {}", *messages_sent_.at(motor_flat_index(motor_id)),
                *messages_received_.at(motor_flat_index(motor_id)));
  }

  // SPDLOG_INFO("Write (ns): " <<prof_utils::duration_ns(stop - start) << "\t"; // Takes around
  // 80us SPDLOG_INFO("Send start (ms): " <<prof_utils::duration_ms(start - debug_start_) << "\t";
  // SPDLOG_INFO("Send end (ms): " <<prof_utils::duration_ms(stop - debug_start_) << "\t";
}

std::vector<int> MotorInterface::micros_since_last_read() const {
  auto now = prof_utils::now();
  std::vector<int> result(time_last_received_.size(), 0);
  for (int i = 0; i < time_last_received_.size(); i++) {
    result[i] = prof_utils::duration_us(now - time_last_received_[i]->load());
  }
  return result;
}

std::vector<int> MotorInterface::receive_counts() const {
  std::vector<int> result;
  for (std::shared_ptr<std::atomic_int> count : messages_received_) {
    result.push_back(*count);
  }
  return result;
}

}  // namespace pupperv3