#include "Dynamic/Env/Event/EventMoveVehicle.hpp"

#include <spdlog/spdlog.h>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

EventMoveVehicle::EventMoveVehicle(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventMoveVehicle::process(Env &env) {
    if(vehicle.state == Vehicle::State::STOPPED) {
        size_t i = vehicle.position.lane.stopped.order_of({vehicle, nullptr});

        vehicle.position.offset = vehicle.position.lane.edge.length - Vehicle::LENGTH * (Length)i;
        vehicle.lastUpdateTime  = env.getTime();
        return;
    }

    Time Dt = env.getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;
    vehicle.lastUpdateTime = env.getTime();

    if(vehicle.position.offset > vehicle.position.lane.edge.length + 0.001) {
        spdlog::warn("Vehicle {} has gone past the end of lane {} after update", vehicle.id, vehicle.position.lane.idAsString());
    }
}
