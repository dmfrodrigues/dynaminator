#pragma once

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Env;

class Lane;

class Position {
   public:
    Lane  &lane;
    Length offset;

    Position(Lane &lane, Length offset);

    Position &operator=(const Position &p);
};
}  // namespace Dynamic::Env
