#include "Dynamic/Env/Event/EventMoveVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Intention Intention;

EventMoveVehicle::EventMoveVehicle(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventMoveVehicle::process(Env &env) const {
    if(vehicle.state == Vehicle::State::STOPPED) {
        vehicle.position.offset = vehicle.position.lane.edge.length;
        return;
    }

    Time Dt = env.getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;

    if(vehicle.position.offset > vehicle.position.lane.edge.length + 0.01) {
        cerr << "EventMoveVehicle::process: WARNING: vehicle " << vehicle.id << " has gone past the end of its lane after update" << endl;
        cerr
            << "    t=" << env.getTime()
            << ", Dt=" << Dt
            << ", offset=" << vehicle.position.offset
            << ", speed=" << vehicle.speed
            << ", length=" << vehicle.position.lane.edge.length
            << endl;
    }

    vehicle.lastUpdateTime = env.getTime();
}
