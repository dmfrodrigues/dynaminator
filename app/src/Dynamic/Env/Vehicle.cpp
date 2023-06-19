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
    Position                position,
    Speed                   speed
):
    Dynamic::Vehicle(vehicle),
    lastUpdateTime(t),
    position(position),
    speed(speed) {}

Vehicle::Policy::Intention Vehicle::pickConnection(Env &env) const {
    return policy->pickConnection(env);
}

void Vehicle::move(Env &env, const Intention &intention) {
    if(intention.connection == Connection::LEAVE) {
        env.removeVehicle(id);
        return;
    } else if(intention.connection == Connection::STOP) {
        return;
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

    Edge &edge = intention.connection.fromLane.edge;
    edge.vehicles.erase(id);

    Edge &toEdge = intention.connection.toLane.edge;

    // clang-format off
    position = {
        intention.lane,
        0
    };
    // clang-format on
    speed = toEdge.calculateSpeed();

    toEdge.vehicles.insert(id);
}
