#include "Dynamic/Env/Vehicle.hpp"

#include <stdexcept>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic::Env;

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

const Connection &Vehicle::pickConnection(Env &env) const {
    return policy->pickConnection(env);
}

void Vehicle::move(Env &env, const Connection &connection) {
    if(connection == Connection::LEAVE) {
        env.removeVehicle(id);
        return;
    } else if(connection == Connection::STOP) {
        return;
    }

    if(position.edge != connection.fromLane.edge)
        throw logic_error("Vehicle::move: vehicle is not at the beginning of edge " + to_string(connection.fromLane.edge.id));

    Edge &edge = *env.edges.at(connection.fromLane.edge.id);
    edge.vehicles.erase(id);

    Edge &toEdge = *env.edges.at(connection.toLane.edge.id);

    // clang-format off
    position = {
        toEdge,
        0
    };
    // clang-format on
    speed = toEdge.calculateSpeed();

    toEdge.vehicles.insert(id);
}
