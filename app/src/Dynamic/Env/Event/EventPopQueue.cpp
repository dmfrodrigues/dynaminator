#include "Dynamic/Env/Event/EventPopQueue.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventPopQueue::EventPopQueue(Time t, Lane &lane):
    Event(t), lane(lane) {}

void EventPopQueue::process(Env &env) const {
    if(lane.stopped.empty())
        return;

    auto [vehicle, intention] = lane.stopped.front();

    if(intention.connection.canPass()) {
        lane.stopped.pop_front();
        vehicle.get().move(env, intention);

        // cerr
        //     << "EventPopQueue::process: vehicle " << vehicle.get().id
        //     << " passed in lane " << lane.edge.id << "_" << lane.index
        //     << ", queue size is now " << lane.stopped.size()
        //     << endl;

        env.pushEvent(make_shared<EventPopQueue>(
            env.getTime() + 1.0,
            lane
        ));
    }
}
