#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <color/color.hpp>
#pragma GCC diagnostic pop

#include "utils/stringifier.hpp"

namespace utils {
template<>
class stringifier<color::rgb<float>> {
   public:
    static std::string toString(const color::rgb<float> &t);
    static color::rgb<float> fromString(const std::string &s);
};

template<>
class stringifier<color::rgba<float>> {
   public:
    static std::string toString(const color::rgba<float> &t);
    static color::rgba<float> fromString(const std::string &s);
};
}  // namespace utils