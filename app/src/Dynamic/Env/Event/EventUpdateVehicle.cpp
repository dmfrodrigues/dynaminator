#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

const Length EPSILON = 1e-3;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    EventMoveVehicle(t_, vehicle_), vehicle(vehicle_) {}

void enqueue(Dynamic::Env::Env &env, Dynamic::Env::Vehicle &vehicle, shared_ptr<Dynamic::Env::Action> action) {
    vehicle.position.offset = vehicle.position.lane.queuePosition();

    vehicle.position.lane.stopped.emplace(vehicle, action);

    vehicle.lastUpdateTime = env.getTime();

    vehicle.speed = 0;

    vehicle.state = Dynamic::Env::Vehicle::State::STOPPED;
}

void EventUpdateVehicle::process(Env &env) {
    EventMoveVehicle::process(env);

    assert(vehicle.position.offset > vehicle.position.lane.queuePosition() - EPSILON);

    // Is at end of edge; enqueue or move to next edge

    auto action = vehicle.pickConnection(env);

    if(action->connection == Connection::LEAVE) {
        if(vehicle.toTAZ.sinks.count(vehicle.position.lane.edge) <= 0) {  // Leaving network at wrong place
            cerr << "[WARN] " << __PRETTY_FUNCTION__ << ": vehicle " << vehicle.id << " is leaving network at wrong place" << endl;
            action->reward(-numeric_limits<Action::Reward>::infinity());
        }

        vehicle.state = Vehicle::State::LEFT;

        assert(vehicle.position.lane.moving.erase(vehicle.id) == 1);

        return;
    } else if(action->connection == Connection::STOP) {
        return;
    }

    Lane &fromLane = action->connection.fromLane;
    Lane &toLane   = action->connection.toLane;

    assert(vehicle.position.lane == fromLane);
    assert(toLane.edge == action->lane.edge);

    assert(fromLane.moving.erase(vehicle.id) == 1);

    // Try to leave current edge
    if(
        !vehicle.position.lane.stopped.empty()
        || !action->connection.canPass()
    ) {
        // Enqueue
        enqueue(env, vehicle, action);

        return;
    }

    Time yieldUntil = action->connection.mustYieldUntil();
    if(yieldUntil > env.getTime()) {
        // Must yield to higher-priority flow; enqueue and wait
        enqueue(env, vehicle, action);

        env.pushEvent(make_shared<EventPopQueue>(
            yieldUntil,
            vehicle.position.lane
        ));

        return;
    }

    // Move to next edge
    vehicle.moveToAnotherEdge(env, action);
}
