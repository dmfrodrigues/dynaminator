#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

const Length EPSILON = 1e-3;

EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle &vehicle_):
    EventMoveVehicle(t_, vehicle_), vehicle(vehicle_) {}

void EventUpdateVehicle::process(Env &env) {
    EventMoveVehicle::process(env);

    if(vehicle.position.offset > vehicle.position.lane.queuePosition() - EPSILON) {
        // Is at end of edge; enqueue or move to next edge

        auto action = vehicle.pickConnection(env);

        if(action->connection == Connection::LEAVE) {
            if(vehicle.toTAZ.sinks.count(vehicle.position.lane.edge) <= 0) {  // Leaving network at wrong place
                cerr << "[WARN] " << __PRETTY_FUNCTION__ << ": vehicle " << vehicle.id << " is leaving network at wrong place" << endl;
                action->reward(-numeric_limits<Action::Reward>::infinity());
            }

            vehicle.state = Vehicle::State::LEFT;

            return;
        } else if(action->connection == Connection::STOP) {
            return;
        }

        assert(vehicle.position.lane == action->connection.fromLane);
        assert(action->connection.toLane.edge == action->lane.edge);

        Lane &fromLane = action->connection.fromLane;
        assert(fromLane.moving.erase(vehicle.id) == 1);

        if(
            !vehicle.position.lane.stopped.empty()
            || !action->connection.canPass()
        ) {
            vehicle.position.lane.stopped.emplace(vehicle, action);

            vehicle.position.offset = vehicle.position.lane.edge.length;
            vehicle.lastUpdateTime  = env.getTime();

            vehicle.speed = 0;

            vehicle.state = Vehicle::State::STOPPED;

            return;
        }

        vehicle.moveToAnotherEdge(env, action);

        return;
    }

    Time newDt   = (vehicle.position.lane.edge.length - vehicle.position.offset) / vehicle.speed;  // TODO: change position.lane.edge.length to position.lane.queuePosition()
    Time tFuture = env.getTime() + newDt;
    env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, vehicle));
}
