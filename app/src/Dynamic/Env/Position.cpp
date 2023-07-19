#include "Dynamic/Env/Position.hpp"

#include <new>

using namespace std;
using namespace Dynamic::Env;

Position::Position(Lane &lane_, Length offset_):
    lane(lane_), offset(offset_) {}

Position &Position::operator=(const Position &p) {
    new(this) Position{p.lane, p.offset};

    return *this;
}
