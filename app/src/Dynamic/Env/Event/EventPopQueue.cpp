#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include <limits>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventDespawnVehicle.hpp"
#include "Dynamic/Env/Event/EventSpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventPopQueue::EventPopQueue(Time t_, Lane &lane_):
    Event(t_),
    lane(lane_) {}

void EventPopQueue::process(Env &env) {
    if(env.getTime() < lane.nextPopTime - TIME_EPSILON)
        return;

    if(lane.stopped.empty())
        return;

    auto p = lane.stopped.front();

    auto &[vehicle, action] = p;

    // clang-format off
    if(
        action->connection.isRed()
        || action->connection.toLane.isFull()
    ) {
        return;
    }
    // clang-format on

    Time yieldUntil = action->connection.mustYieldUntil();
    if(yieldUntil > env.getTime()) {
        // Vehicle must yield
        env.pushEvent(make_shared<EventPopQueue>(
            yieldUntil,
            lane
        ));

        return;
    }

    // Move vehicle at front of queue
    lane.stopped.pop();

    vehicle.get().moveToAnotherEdge(env, action);

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
