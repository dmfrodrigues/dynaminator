#pragma once

#include <functional>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    enum class State : int {
        MOVING  = 1,  /// @brief Vehicle is moving
        STOPPED = 2,  /// @brief Vehicle is stopped at a queue
        LEFT    = 3   /// @brief Vehicle has left the simulation
    };

    static const Length LENGTH;

    Time     lastUpdateTime;
    Position position;
    Speed    speed;
    State    state;

    std::shared_ptr<Action> prevAction;

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

    std::shared_ptr<Action> pickConnection(Env &env) const;

    void moveToAnotherEdge(Env &env, std::shared_ptr<Action> action);

    bool operator<(const Dynamic::Env::Vehicle &other) const;
    bool operator==(const Dynamic::Env::Vehicle &other) const;
};
}  // namespace Dynamic::Env
