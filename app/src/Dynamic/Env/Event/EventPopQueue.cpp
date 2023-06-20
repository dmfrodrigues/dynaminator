#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventPopQueue::EventPopQueue(Time t, Lane &lane):
    Event(t), lane(lane) {}

void EventPopQueue::process(Env &env) {
    if(lane.stopped.empty())
        return;

    auto p = lane.stopped.front();

    auto &[vehicle, action] = p;

    if(action->connection.canPass()) {
        lane.stopped.pop();

        Vehicle &veh = vehicle;

        // clang-format off
        veh.position = {
            action->lane,
            0
        };
        // clang-format on
        veh.position.lane.moving.insert(veh.id);
        veh.speed          = veh.position.lane.calculateSpeed();
        veh.state          = Vehicle::State::MOVING;
        veh.lastUpdateTime = env.getTime();

        env.pushEvent(make_shared<EventUpdateVehicle>(
            env.getTime(),
            veh
        ));

        env.pushEvent(make_shared<EventPopQueue>(
            env.getTime() + 0.01,
            lane
        ));
    }
}
