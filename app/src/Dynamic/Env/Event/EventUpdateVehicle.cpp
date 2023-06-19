#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Intention Intention;

const Length EPSILON = 1e-3;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    EventMoveVehicle(t_, vehicle_), vehicle(vehicle_) {}

void EventUpdateVehicle::process(Env &env) const {
    EventMoveVehicle::process(env);

    bool newEvent = true;

    if(vehicle.position.offset > vehicle.position.lane.edge.length - EPSILON) {
        // Is at end of edge; enqueue or move to next edge
        // TODO: enqueue

        Intention intention = vehicle.pickConnection(env);

        newEvent = vehicle.move(env, intention);
    } else {
        cerr << "Warning: creating EventUpdateVehicle after not changing the env state in a significant way" << endl;
        cerr << "    vehicle: " << vehicle.id << endl;
        cerr << "    t: " << getTime() << ", env.t: " << env.getTime() << endl;
        cerr << "    offset: " << vehicle.position.offset << ", length: " << vehicle.position.lane.edge.length << endl;
    }

    if(newEvent) {
        Time newDt   = (vehicle.position.lane.edge.length - vehicle.position.offset) / vehicle.speed;
        Time tFuture = env.getTime() + newDt;
        env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, vehicle));
    }
}
