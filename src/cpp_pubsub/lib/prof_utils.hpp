#pragma once

#define duration_ns(t) chrono::duration_cast<chrono::nanoseconds>(t).count()
#define duration_ms(t) chrono::duration_cast<chrono::microseconds>(t).count()
#define time_now() chrono::high_resolution_clock::now()