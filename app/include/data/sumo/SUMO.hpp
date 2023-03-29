#pragma once

#include <string>
#include <vector>

#include "geo/Coord.hpp"

namespace SUMO {
typedef int Time;
typedef std::vector<Coord> Shape;
Shape stringToShape(const std::string &s);
}  // namespace SUMO
