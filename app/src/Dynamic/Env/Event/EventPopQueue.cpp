#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

const Dynamic::Time TIME_EPSILON = 1e-6;

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

    Time yieldUntil = action->connection.mustYieldUntil();
    if(yieldUntil > env.getTime()) {
        // Vehicle must yield
        env.pushEvent(make_shared<EventPopQueue>(
            yieldUntil,
            lane
        ));

        return;
    }

    if(action->connection.canPass()) {
        // Move vehicle at front of queue
        lane.stopped.pop();

        vehicle.get().moveToAnotherEdge(env, action);

        /*
         * Process next waiting vehicle (instantiate vehicle or get vehicle from
         * previous queue)
         */
        lane.processNextWaitingVehicle(env);

        // TODO: check if EventPopQueue should only be created if !stopped.empty()
        // Schedule next EventPopQueue
        Time tFuture = env.getTime() + Lane::JUNCTION_PERIOD;
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
}
