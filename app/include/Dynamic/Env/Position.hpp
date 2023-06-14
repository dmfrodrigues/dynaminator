#pragma once

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Env;

class Edge;

class Position {
   public:
    const Edge &edge;
    Length      offset;

    Position &operator=(const Position &p);
};
}  // namespace Dynamic::Env
