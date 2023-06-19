#pragma once

#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    enum class State {
        MOVING,
        STOPPED,
    };

    Time     lastUpdateTime;
    Position position;
    Speed    speed;
    State    state;

    Vehicle(
        const Dynamic::Vehicle &vehicle,
        Time                    t,
        Position                position,
        Speed                   speed,
        State                   state
    );

    Vehicle::Policy::Intention pickConnection(Env &env) const;

    bool move(Env &env, const Vehicle::Policy::Intention &connection);
};
}  // namespace Dynamic::Env
