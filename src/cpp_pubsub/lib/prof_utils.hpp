#pragma once

#define duration_ns(t) std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()
#define duration_ms(t) std::chrono::duration_cast<std::chrono::microseconds>(t).count()
#define time_now() std::chrono::high_resolution_clock::now()