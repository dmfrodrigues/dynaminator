#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wconversion"
#include <color/color.hpp>
#pragma GCC diagnostic pop

#include "utils/stringify.hpp"

namespace utils::stringify {
template<>
class stringify<color::rgb<float>> {
   public:
    static std::string       toString(const color::rgb<float> &t);
    static color::rgb<float> fromString(const std::string &s);
};

template<>
class stringify<color::rgba<float>> {
   public:
    static std::string        toString(const color::rgba<float> &t);
    static color::rgba<float> fromString(const std::string &s);
};
}  // namespace utils::stringify
