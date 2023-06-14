#pragma once

#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace SUMO {
struct NetworkTAZs {
   public:
    Network &network;
    TAZs    &tazs;
};
}  // namespace SUMO
