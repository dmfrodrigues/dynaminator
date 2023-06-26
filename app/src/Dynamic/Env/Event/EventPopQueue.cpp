#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventPopQueue::EventPopQueue(Time t_, Lane &lane_):
    Event(t_),
    lane(lane_) {}

void EventPopQueue::process(Env &env) {
    if(lane.stopped.empty())
        return;

    auto p = lane.stopped.front();

    auto &[vehicle, action] = p;

    if(action->connection.canPass()) {
        // Move vehicle at front of queue
        lane.stopped.pop();

        Vehicle &veh = vehicle;

        veh.moveToAnotherEdge(env, action);

        /*
         * TODO: I believe it is better to calculate Dt and predict the time of
         * the next event, instead of calling EventUpdateVehicle just for it to
         * create a new EventUpdateVehicle at the right time.
         */
        env.pushEvent(make_shared<EventUpdateVehicle>(
            env.getTime(),
            veh
        ));

        /*
         * Process next waiting vehicle (instantiate vehicle or get vehicle from
         * previous queue)
         */
        lane.processNextWaitingVehicle(env);

        // Schedule next EventPopQueue
        env.pushEvent(make_shared<EventPopQueue>(
            env.getTime() + Lane::JUNCTION_PERIOD,
            lane
        ));
    }
}
