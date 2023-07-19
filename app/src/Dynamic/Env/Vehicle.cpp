#include "Dynamic/Env/Vehicle.hpp"

#include <limits>
#include <stdexcept>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Policy/Policy.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunctionGreedy.hpp"

using namespace std;
using namespace Dynamic::Env;

const Dynamic::Length Vehicle::LENGTH = 6.52;

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

    assert(!action->connection.isRed());
    assert(!action->connection.toLane.isFull());

    // Reward
    if(prevAction) {
        Action::Reward r = env.rewardFunction(env, *this);

        prevAction->reward(r);
    }

    action->connection.lastUsed = env.getTime();

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

    size_t N = position.lane.moving.size();
    if(N >= 500 && N % 10 == 0) {
        cerr
            << "[WARN][t=" << env.getTime() << "]: "
            << "lane " << position.lane.idAsString()
            << " has " << N
            << " moving vehicles" << endl;
    }

    Time Dt      = (position.lane.queuePosition() - position.offset) / speed;
    Dt           = max(Dt, 0.0);
    Time tFuture = env.getTime() + Dt;
    env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, *this));
}

bool Dynamic::Env::Vehicle::operator==(const Dynamic::Env::Vehicle &other) const {
    return Dynamic::Vehicle::operator==(other);
}
