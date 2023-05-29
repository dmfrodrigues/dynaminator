#pragma once

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "Vector2.hpp"
#pragma GCC diagnostic pop

#include "utils/stringify.hpp"

/**
 * @brief SUMO (Simulation of Urban MObility) is an open source, portable,
 * microscopic and continuous multi-modal traffic simulation package designed to
 * handle large networks.
 *
 * SUMO is developed by the German Aerospace Center and community users. It has
 * been freely available as open-source since 2001, and since 2017 it is an
 * Eclipse Foundation project. (Wikipedia)
 */
namespace SUMO {
typedef std::string ID;
typedef double      Time;
typedef double      Length;
typedef double      Speed;
typedef ssize_t     Index;
typedef Vector2     Coord;

typedef std::vector<Coord> Shape;
}  // namespace SUMO

namespace std {
template<>
struct hash<SUMO::Coord> {
    size_t operator()(const SUMO::Coord &v) const;
};
}  // namespace std

namespace utils::stringify {
template<>
class stringify<SUMO::Coord> {
   public:
    static SUMO::Coord fromString(const std::string &s);
    static std::string toString(const SUMO::Coord &t);
};
}  // namespace utils::stringify
