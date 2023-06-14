#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventUpdateVehicle::process(Env &env) const {
    Time Dt = env.getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;

    vehicle.lastUpdateTime = env.getTime();
}
