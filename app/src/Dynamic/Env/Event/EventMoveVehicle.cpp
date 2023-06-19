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
    Time Dt = env.getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;

    vehicle.lastUpdateTime = env.getTime();
}
