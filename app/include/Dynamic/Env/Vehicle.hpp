#pragma once

#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    Time     lastUpdateTime;
    Position position;
    Speed    speed;

    Vehicle(
        const Dynamic::Vehicle &vehicle,
        Time t,
        Position position,
        Speed speed
    );

    const Connection &pickConnection(Env &env) const;

    void move(Env &env, const Connection &connection);
};
}  // namespace Dynamic::Env
