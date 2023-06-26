#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

const Length EPSILON = 1e-3;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    EventMoveVehicle(t_, vehicle_), vehicle(vehicle_) {}

void EventUpdateVehicle::process(Env &env) {
    EventMoveVehicle::process(env);

    if(vehicle.position.offset > vehicle.position.lane.queuePosition() - EPSILON) {
        // Is at end of edge; enqueue or move to next edge

        auto action = vehicle.pickConnection(env);

        vehicle.move(env, action);

        return;
    }

    Time newDt   = (vehicle.position.lane.edge.length - vehicle.position.offset) / vehicle.speed;  // TODO: change position.lane.edge.length to position.lane.queuePosition()
    Time tFuture = env.getTime() + newDt;
    env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, vehicle));
}
