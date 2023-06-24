/*
 * While this program worked on Linux, it never quite worked on Mac
 * My 1 second timer became a 1.6 second timer.
 * And a 0.8s latency would occur every 1s interval
 *
 * Compile with g++ -std=c++17 -O2 -o mac_serial_test mac_serial_test.cc
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

constexpr int kMillisPerPeriod = 5000;

int main() {
  std::cout << "Begin main...\n";
  std::ifstream input("/dev/ttyACM0");  // Change for Mac
  std::cout << "Opened PICO dev port\n";
  std::string line;
  int count = 0;
  auto last_print = std::chrono::high_resolution_clock::now();
  auto last_line = std::chrono::high_resolution_clock::now();
  std::vector<int> micros_btn_getline;
  micros_btn_getline.reserve(1000);
  long min_time = std::numeric_limits<long>::max();
  long max_time = std::numeric_limits<long>::min();
  int num_zeros = 0;
  while (true) {
    if (std::getline(input, line, '\n')) {
      auto getline = std::chrono::high_resolution_clock::now();

      if (line.size() > 0) {
        if (line.size() != 48) {
          std::cout << "ERROR. Line of length: " << line.size() << "\n";
          continue;
        } else {
          count++;

          auto duration =
              std::chrono::duration_cast<std::chrono::microseconds>(getline - last_line);
          last_line = getline;
          long micros = duration.count();
          //   std::cout << "Microseconds since last message: " << micros << " (us)\n";
          micros_btn_getline.push_back(micros);
          min_time = std::min(min_time, micros);
          max_time = std::max(max_time, micros);
          if (micros < 5) {
            num_zeros++;
          }
        }
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