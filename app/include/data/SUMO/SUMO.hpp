#pragma once

#include <string>
#include <vector>

#include "Geo/Coord.hpp"
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
typedef ssize_t     Index;

typedef std::vector<Geo::Coord> Shape;
}  // namespace SUMO
