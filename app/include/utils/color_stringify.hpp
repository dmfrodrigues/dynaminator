#pragma once

#include <color/color.hpp>

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
