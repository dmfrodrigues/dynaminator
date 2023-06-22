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

    Time enteredLane;

    Vehicle(
        const Dynamic::Vehicle &vehicle,
        Time                    t,
        Position                position,
        Speed                   speed,
        State                   state
    );

    std::shared_ptr<Vehicle::Policy::Action> pickConnection(Env &env) const;

    void moveToAnotherEdge(Env &env, Vehicle::Policy::Action &action);

    bool move(Env &env, std::shared_ptr<Vehicle::Policy::Action> &connection);

    bool operator<(const Dynamic::Env::Vehicle &other) const;
};
}  // namespace Dynamic::Env
