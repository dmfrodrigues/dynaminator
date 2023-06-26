#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

// TODO: change this value to something that makes more sense, like 1s.
/**
 * @brief Frequency at which vehicles leave a queue.
 */
const double JUNCTION_FREQUENCY = 1.0 / (1600.0 / 60.0 / 60.0);

EventPopQueue::EventPopQueue(Time t_, Lane &lane_):
    Event(t_), lane(lane_) {}

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

        env.pushEvent(make_shared<EventUpdateVehicle>(
            env.getTime(),
            veh
        ));

        // Instantiate uninstantiated vehicle
        if(!lane.uninstantiated.empty()) {
            Dynamic::Vehicle instantiatedVehicle = lane.uninstantiated.front();
            lane.uninstantiated.pop();

            EventTrySpawnVehicle event(
                env.getTime(),
                instantiatedVehicle
            );
            event.process(env);
        }

        // Schedule next EventPopQueue
        env.pushEvent(make_shared<EventPopQueue>(
            env.getTime() + JUNCTION_FREQUENCY,
            lane
        ));
    }
}
