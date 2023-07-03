#include "Dynamic/Env/Event/EventDespawnVehicle.hpp"

#include <stdexcept>

#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventDespawnVehicle::EventDespawnVehicle(Time t, Vehicle::ID vehicleID_):
    Event(t),
    vehicleID(vehicleID_) {}

void EventDespawnVehicle::process(Env &env) {
    Vehicle *vehiclePtr = nullptr;
    try {
        vehiclePtr = &env.getVehicle(vehicleID);
    } catch(const out_of_range &e) {
        // Vehicle does not exist
        return;
    }
    Vehicle &vehicle = *vehiclePtr;

    Time waitingFor = env.getTime() - vehicle.lastUpdateTime;
    if(waitingFor < env.getDespawnTime()) {
        // Vehicle has been updated since the event was scheduled
        return;
    }

    cerr << "Despawning vehicle " << vehicle.id << " at time " << env.getTime()
         << ", lastUpdateTime is " << vehicle.lastUpdateTime
         << ", vehicle is at lane " << vehicle.position.lane.idAsString()
         << endl;

    // // Vehicle has been waiting for too long
    // EventUpdateVehicle event(env.getTime(), vehicle);
    // event.process(env);

    assert(vehicle.state == Vehicle::State::STOPPED);
    assert(vehicle.position.lane.stopped.front().first.get() == vehicle);

    // assert(vehicle.position.lane.stopped.erase({vehicle, nullptr}) == 1);
    vehicle.position.lane.stopped.pop();
    vehicle.state = Vehicle::State::LEFT;
    ++env.numberOfDespawnedVehicles();

    env.pushEvent(make_shared<EventPopQueue>(
        env.getTime(),
        vehicle.position.lane
    ));

    if(vehicle.position.lane.stopped.size() > 0) {
        Vehicle &frontVehicle       = vehicle.position.lane.stopped.front().first.get();
        frontVehicle.lastUpdateTime = env.getTime();
        env.pushEvent(make_shared<EventDespawnVehicle>(
            env.getTime() + env.getDespawnTime(),
            frontVehicle.id
        ));
    }
}
