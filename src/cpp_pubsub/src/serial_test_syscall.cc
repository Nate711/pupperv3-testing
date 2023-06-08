/*
 * Run from build folder to get permissions
 *
 */

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <errno.h>  // Error integer and strerror() function
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

constexpr int kMillisPerPeriod = 1000;
// constexpr unsigned int kSerialBaud = (speed_t)B115200;
constexpr unsigned int kSerialBaud = (speed_t)B1000000;

const char* serial_device = "/dev/ttyUSB0";

int main() {
  std::cout << "Begin main...\n";
  int file_descriptor;
  char data[255];
  struct termios tty;
  file_descriptor = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
  if (file_descriptor < 0) {
    std::cout << "ERROR OPENING TTY\n";
    printf("Error %i from open: %s\n", errno, std::strerror(errno));
    return -1;
  }
  if (tcgetattr(file_descriptor, &tty) != 0) {
    std::cout << "ERROR IN TCGETATTR\n";
    printf("Error %i from tcgetattr: %s\n", errno, std::strerror(errno));
    return -1;
  }
  cfsetospeed(&tty, kSerialBaud);
  cfsetispeed(&tty, kSerialBaud);

  // Default on Arduino is 8 bit, 1 stop bit, no parity
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;

  if (tcsetattr(file_descriptor, TCSANOW, &tty) != 0) {
    std::cout << "ERROR IN SETTING SERIAL ATTRIBUTES\n";
    return -1;
  }
  std::cout << "Opened PICO dev port\n";

  std::string line;
  int count = 0;
  auto last_print = std::chrono::high_resolution_clock::now();
  auto last_line = std::chrono::high_resolution_clock::now();
  auto last_full_message = std::chrono::high_resolution_clock::now();
  std::vector<int> micros_btn_getline;
  micros_btn_getline.reserve(1000);
  long min_time = std::numeric_limits<long>::max();
  long max_time = std::numeric_limits<long>::min();
  int num_zeros = 0;
  while (true) {
    auto read_start = std::chrono::high_resolution_clock::now();
    int n_read = read(file_descriptor, data, sizeof(data));
    auto read_end = std::chrono::high_resolution_clock::now();
    if (n_read <= 0) {
      // std::cout << "n_read <= 0: " << n_read << "\n";
    } else {
      auto read_duration =
          std::chrono::duration_cast<std::chrono::microseconds>(read_end - read_start);
      std::cout << "read syscall took: " << read_duration.count() << " (us)\n";
      std::cout << "n_read: " << n_read << "\n";
      if (n_read == 49) {
        count++;
        auto now = std::chrono::high_resolution_clock::now();
        auto since_last_full_message =
            std::chrono::duration_cast<std::chrono::microseconds>(now - last_full_message);
        auto since_last_full_message_micros = since_last_full_message.count();
        std::cout << "Since last full message: " << since_last_full_message_micros << " (us)\n";
        micros_btn_getline.push_back(since_last_full_message_micros);
        min_time = std::min(min_time, since_last_full_message_micros);
        max_time = std::max(max_time, since_last_full_message_micros);
        if (since_last_full_message_micros < 5) {
          num_zeros++;
        }
        last_full_message = now;
      }
    }

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print);

    if (elapsed_time.count() >= kMillisPerPeriod) {
      std::cout << "Received " << count
                << " messages. Per second: " << count * 1000 / kMillisPerPeriod
                << " Max: " << max_time << " (us) "
                << " Min: " << min_time << " (us) "
                << "Number of instantaneous getlines: " << num_zeros << std::endl;
      count = 0;
      last_print = now;
      min_time = std::numeric_limits<long>::max();
      max_time = std::numeric_limits<long>::min();
      micros_btn_getline.clear();
      num_zeros = 0;
    }
  }

  return 0;
}