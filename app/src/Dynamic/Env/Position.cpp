#include "Dynamic/Env/Position.hpp"

#include <new>

using namespace std;
using namespace Dynamic::Env;

Position &Position::operator=(const Position &p) {
    new(this) Position{p.edge, p.offset};

    return *this;
}
