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

    if(position.lane.edge != intention.connection.fromLane.edge)
        throw logic_error("Vehicle::move: vehicle is not at the beginning of edge " + to_string(intention.connection.fromLane.edge.id));

    Edge &edge = *env.edges.at(intention.connection.fromLane.edge.id);
    edge.vehicles.erase(id);

    Edge &toEdge = *env.edges.at(intention.connection.toLane.edge.id);

    // clang-format off
    position = {
        intention.connection.toLane,
        0
    };
    // clang-format on
    speed = toEdge.calculateSpeed();

    toEdge.vehicles.insert(id);
}
