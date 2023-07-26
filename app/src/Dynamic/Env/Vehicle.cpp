#include "Dynamic/Env/Vehicle.hpp"

#include <spdlog/spdlog.h>

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

Vehicle::Path::iterator Vehicle::Path::begin() {
    return v.begin();
}

Vehicle::Path::iterator Vehicle::Path::end() {
    return v.end();
}

Vehicle::Path::const_iterator Vehicle::Path::begin() const {
    return v.begin();
}

Vehicle::Path::const_iterator Vehicle::Path::end() const {
    return v.end();
}

const pair<Dynamic::Time, reference_wrapper<const Lane>> &Vehicle::Path::front() const {
    return v.front();
}

const pair<Dynamic::Time, reference_wrapper<const Lane>> &Vehicle::Path::back() const {
    return v.back();
}

void Vehicle::Path::emplace_connection(Time t, const Connection &connection) {
    if(!empty()) {
        assert(back().first <= t);
        assert(back().second.get() == connection.fromLane);
    }

    v.emplace_back(t, connection.toLane);
}

void Vehicle::Path::emplace_lane(Time t, const Lane &lane) {
    if(!empty()) {
        assert(back().first <= t);
        assert(back().second.get().edge == lane.edge);
    }

    v.emplace_back(t, lane);
    ++ms[lane];
}

size_t Vehicle::Path::count(const Lane &lane) const {
    auto it = ms.find(lane);
    return (it != ms.end() ? it->second : 0);
}

size_t Vehicle::Path::size() const {
    return v.size();
}

bool Vehicle::Path::empty() const {
    return v.empty();
}

Vehicle::Vehicle(
    const Dynamic::Vehicle &vehicle,
    Time                    t,
    Position                position_,
    Speed                   speed_,
    State                   state_
):
    Dynamic::Vehicle(vehicle),
    lastUpdateTime(t),
    lastMoveTime(t),
    position(position_),
    speed(speed_),
    state(state_),
    enteredLane(t) {
    path.emplace_lane(t, position.lane);
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

    path.emplace_connection(env.getTime(), action->connection);

    // clang-format off
    position = {
        action->lane,
        0
    };
    // clang-format on
    position.lane.moving.insert(id);
    path.emplace_lane(env.getTime(), position.lane);
    speed          = position.lane.calculateSpeed();
    state          = Vehicle::State::MOVING;
    lastUpdateTime = env.getTime();
    lastMoveTime   = env.getTime();
    enteredLane    = env.getTime();

    prevAction = action;

    size_t N = position.lane.moving.size();
    if(N >= 500 && N % 10 == 0) {
        spdlog::warn(
            "[t={}] lane {} has {} moving vehicles",
            env.getTime(),
            position.lane.idAsString(),
            N
        );
    }

    Time Dt      = (position.lane.queuePosition() - position.offset) / speed;
    Dt           = max(Dt, 0.0);
    Time tFuture = env.getTime() + Dt;
    env.pushEvent(make_shared<EventUpdateVehicle>(tFuture, *this));
}

bool Dynamic::Env::Vehicle::operator==(const Dynamic::Env::Vehicle &other) const {
    return Dynamic::Vehicle::operator==(other);
}
