#include "Dynamic/Env/Vehicle.hpp"

#include <limits>
#include <stdexcept>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Policy/Policy.hpp"

using namespace std;
using namespace Dynamic::Env;

Vehicle::Vehicle(
    const Dynamic::Vehicle &vehicle,
    Time                    t,
    Position                position_,
    Speed                   speed_,
    State                   state_
):
    Dynamic::Vehicle(vehicle),
    lastUpdateTime(t),
    position(position_),
    speed(speed_),
    state(state_),
    enteredLane(t) {
    path.emplace_back(t, position.lane);
}

shared_ptr<Action> Vehicle::pickConnection(Env &env) const {
    return policy->pickConnection(env);
}

void Vehicle::moveToAnotherEdge(Env &env, shared_ptr<Action> action) {
    assert(position.lane == action->connection.fromLane);

    // Reward
    /**
     * FIXME: this part is very ooga booga, and not generic at all; ideally we
     * would allow using a generic reward function.
     */
    if(prevAction) {
        Time leftLane = env.getTime();
        assert(enteredLane == path.back().first);
        Time t = leftLane - enteredLane;

        Action::Reward r = -t;

        prevAction->reward(r);
    }

    path.emplace_back(env.getTime(), action->connection.toLane);

    // clang-format off
    position = {
        action->lane,
        0
    };
    // clang-format on
    position.lane.moving.insert(id);
    path.emplace_back(env.getTime(), position.lane);
    speed          = position.lane.calculateSpeed();
    state          = Vehicle::State::MOVING;
    lastUpdateTime = env.getTime();
    enteredLane    = env.getTime();

    prevAction = action;
}

bool Vehicle::move(Env &env, shared_ptr<Action> &action) {
    if(action->connection == Connection::LEAVE) {
        if(toTAZ.sinks.count(position.lane.edge) <= 0) {  // Leaving network at wrong place
            cerr << "[WARN] " << __PRETTY_FUNCTION__ << ": vehicle " << id << " is leaving network at wrong place" << endl;
            action->reward(-numeric_limits<Action::Reward>::infinity());
        }

        state = State::LEFT;

        return false;
    } else if(action->connection == Connection::STOP) {
        return false;
    }

    if(position.lane != action->connection.fromLane) {
        // clang-format off
        throw logic_error(
            "Vehicle::move: vehicle " + to_string(id) +
            " is at lane " + to_string(position.lane.edge.id) + "_" + to_string(position.lane.index) +
            ", but connection " + to_string(action->connection.id) +
            " starts at lane " + to_string(action->connection.fromLane.edge.id) + "_" + to_string(action->connection.fromLane.index)
        );
        // clang-format on
    }

    if(action->connection.toLane.edge != action->lane.edge)
        throw logic_error("Vehicle::move: intention destination lane " + to_string(action->lane.edge.id) + " is not on same edge as connection.to " + to_string(action->connection.toLane.edge.id));

    Lane &fromLane = action->connection.fromLane;
    assert(fromLane.moving.erase(id) == 1);

    if(
        !position.lane.stopped.empty()
        || !action->connection.canPass()
    ) {
        position.lane.stopped.emplace(*this, action);

        position.offset = position.lane.edge.length;
        lastUpdateTime  = env.getTime();

        speed = 0;

        state = State::STOPPED;

        return false;
    }

    moveToAnotherEdge(env, action);

    return true;
}

bool Dynamic::Env::Vehicle::operator<(const Dynamic::Env::Vehicle &other) const {
    return Dynamic::Vehicle::operator<(other);
}
