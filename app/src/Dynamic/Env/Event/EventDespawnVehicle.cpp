#include "Dynamic/Env/Event/EventDespawnVehicle.hpp"

#include <spdlog/spdlog.h>

#include <stdexcept>

#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventDespawnVehicle::EventDespawnVehicle(Time t_, Vehicle::ID vehicleID_):
    Event(t_),
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

    spdlog::info(
        "[t={}] Despawning vehicle {} at lane {}",
        env.getTime(),
        vehicle.id,
        vehicle.position.lane.idAsString()
    );

    assert(vehicle.state == Vehicle::State::STOPPED);
    assert(vehicle.position.lane.stopped.front().first.get() == vehicle);

    Lane &lane = vehicle.position.lane;

    // Actually despawn
    if(lane.stopped.empty())
        return;

    auto p = lane.stopped.front();

    // Move vehicle at front of queue
    lane.stopped.pop();

    vehicle.state = Vehicle::State::LEFT;
    ++env.numberOfDespawnedVehicles();

    // Change lastUpdateTime for despawning
    // clang-format off
    if(
        env.getDespawnTime() < numeric_limits<Time>::infinity() &&
        lane.stopped.size() > 0
    ) {
        // clang-format on
        Vehicle &frontVehicle       = lane.stopped.front().first.get();
        frontVehicle.lastUpdateTime = env.getTime();
        env.pushEvent(make_shared<EventDespawnVehicle>(
            env.getTime() + env.getDespawnTime(),
            frontVehicle.id
        ));
    }

    /*
     * Process next waiting vehicle (instantiate vehicle or get vehicle from
     * previous queue)
     */
    lane.processNextWaitingVehicle(env);

    // TODO: check if EventPopQueue should only be created if !stopped.empty()
    // Schedule next EventPopQueue
    Time tFuture = env.getTime() + Lane::QUEUE_PERIOD;
    env.pushEvent(make_shared<EventPopQueue>(
        tFuture,
        lane
    ));
    lane.nextPopTime = tFuture;

    // If queue dissipated, pull from every lane.
    if(lane.stopped.empty()) {
        for(Connection &connection: lane.getIncomingConnections()) {
            Lane &prevLane = connection.fromLane;
            env.pushEvent(make_shared<EventPopQueue>(
                tFuture,
                prevLane
            ));
        }
    }
}
