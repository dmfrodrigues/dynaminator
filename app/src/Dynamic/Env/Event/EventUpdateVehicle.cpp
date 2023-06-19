#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Intention Intention;

const Length EPSILON = 1e-3;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventUpdateVehicle::process(Env &env) const {
    Time Dt = env.getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;

    vehicle.lastUpdateTime = env.getTime();

    if(vehicle.position.offset > vehicle.position.lane.edge.length - EPSILON) {
        // Is at end of edge; enqueue or move to next edge
        // TODO: enqueue

        Intention intention = vehicle.pickConnection(env);

        if(vehicle.move(env, intention)) {
            Time newDt   = (vehicle.position.lane.edge.length - vehicle.position.offset) / vehicle.speed;
            Time tFuture = env.getTime() + newDt;
            env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, vehicle));
        }
    } else {
        cerr << "Warning: creating EventUpdateVehicle after not changing the env state in a significant way" << endl;
        cerr << "    vehicle: " << vehicle.id << endl;
        cerr << "    t: " << getTime() << ", env.t: " << env.getTime() << endl;
        cerr << "    offset: " << vehicle.position.offset << ", length: " << vehicle.position.lane.edge.length << endl;

        Time newDt   = (vehicle.position.lane.edge.length - vehicle.position.offset) / vehicle.speed;
        Time tFuture = env.getTime() + newDt;
        env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, vehicle));
    }
}
