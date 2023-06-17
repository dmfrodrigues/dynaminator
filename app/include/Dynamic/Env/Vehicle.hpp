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
        Time                    t,
        Position                position,
        Speed                   speed
    );

    Vehicle::Policy::Intention pickConnection(Env &env) const;

    void move(Env &env, const Vehicle::Policy::Intention &connection);
};
}  // namespace Dynamic::Env
