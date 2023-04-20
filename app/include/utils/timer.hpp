#pragma once

#include <chrono>

namespace utils {
class timer {
    typedef std::chrono::high_resolution_clock clock;

    clock::time_point start;

   public:
    timer();
    void   tick();
    double tock() const;
};
}  // namespace utils
