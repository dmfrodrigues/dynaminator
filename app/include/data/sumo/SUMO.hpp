#pragma once

#include <string>
#include <vector>

#include "geo/Coord.hpp"
#include "utils/stringifier.hpp"

namespace SUMO {
typedef int Time;
typedef int Index;
typedef std::vector<Coord> Shape;
}  // namespace SUMO

namespace utils {
template <>
class stringifier<SUMO::Shape> {
   public:
    static SUMO::Shape fromString(const std::string &s);
};
}  // namespace utils
