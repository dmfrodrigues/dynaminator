#include "utils/timer.hpp"

using namespace std;
using namespace utils;

timer::timer():
    start(clock::now()) {}

void timer::tick() {
    start = clock::now();
}

double timer::tock() const {
    // clang-format off
    return (double)std::chrono::duration_cast<std::chrono::nanoseconds>(
        clock::now() - start
    ).count() * 1.0e-9;
    // clang-format on
}
