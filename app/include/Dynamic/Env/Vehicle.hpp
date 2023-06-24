#pragma once

#include <functional>

#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    enum class State {
        MOVING,   /// @brief Vehicle is moving
        STOPPED,  /// @brief Vehicle is stopped at a queue
        LEFT      /// @brief Vehicle has left the simulation
    };

    Time     lastUpdateTime;
    Position position;
    Speed    speed;
    State    state;

    std::shared_ptr<Policy::Action> prevAction;

    Time enteredLane;

    typedef std::vector<std::pair<Time, std::reference_wrapper<Lane>>> Path;

    Path path;

    Vehicle(
        const Dynamic::Vehicle &vehicle,
        Time                    t,
        Position                position,
        Speed                   speed,
        State                   state
    );

    std::shared_ptr<Vehicle::Policy::Action> pickConnection(Env &env) const;

    void moveToAnotherEdge(Env &env, std::shared_ptr<Vehicle::Policy::Action> action);

    bool move(Env &env, std::shared_ptr<Vehicle::Policy::Action> &connection);

    bool operator<(const Dynamic::Env::Vehicle &other) const;
};
}  // namespace Dynamic::Env
