#pragma once

#include <functional>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    enum class State {
        MOVING,   /// @brief Vehicle is moving
        STOPPED,  /// @brief Vehicle is stopped at a queue
        LEFT      /// @brief Vehicle has left the simulation
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

    void move(Env &env, std::shared_ptr<Action> &connection);

    bool operator<(const Dynamic::Env::Vehicle &other) const;
};
}  // namespace Dynamic::Env
