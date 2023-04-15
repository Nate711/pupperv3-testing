#pragma once
namespace prof_utils {
template <typename T>
inline long duration_ns(T t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
}

template <typename T>
inline long duration_ms(T t) {
  return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
}

inline std::chrono::time_point<std::chrono::high_resolution_clock> now() {
  return std::chrono::high_resolution_clock::now();
}
}  // namespace prof_utils