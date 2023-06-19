#include "Dynamic/Env/Vehicle.hpp"

#include <stdexcept>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Intention Intention;

Vehicle::Vehicle(
    const Dynamic::Vehicle &vehicle,
    Time                    t,
    Position                position_,
    Speed                   speed_,
    State                   state_
):
    Dynamic::Vehicle(vehicle),
    lastUpdateTime(t),
    position(position_),
    speed(speed_),
    state(state_) {}

Vehicle::Policy::Intention Vehicle::pickConnection(Env &env) const {
    return policy->pickConnection(env);
}

bool Vehicle::move(Env &env, const Intention &intention) {
    if(intention.connection == Connection::LEAVE) {
        env.removeVehicle(id);
        return false;
    } else if(intention.connection == Connection::STOP) {
        return false;
    }

    if(position.lane != intention.connection.fromLane) {
        // clang-format off
        throw logic_error(
            "Vehicle::move: vehicle " + to_string(id) +
            " is at lane " + to_string(position.lane.edge.id) + "_" + to_string(position.lane.index) +
            ", but connection " + to_string(intention.connection.id) +
            " starts at lane " + to_string(intention.connection.fromLane.edge.id) + "_" + to_string(intention.connection.fromLane.index)
        );
        // clang-format on
    }

    if(intention.connection.toLane.edge != intention.lane.edge)
        throw logic_error("Vehicle::move: intention destination lane " + to_string(intention.lane.edge.id) + " is not on same edge as connection.to " + to_string(intention.connection.toLane.edge.id));

    Lane &fromLane = intention.connection.fromLane;
    fromLane.moving.erase(id);

    if(
        !position.lane.stopped.empty()
        || !intention.connection.canPass()
    ) {
        position.lane.stopped.emplace_back(*this, intention);

        position.offset = position.lane.edge.length;

        speed = 0;

        state = State::STOPPED;

        return false;
    }

    Lane &toLane = intention.lane;

    // clang-format off
    position = {
        toLane,
        0
    };
    // clang-format on
    speed = toLane.calculateSpeed();

    toLane.moving.insert(id);

    return true;
}
