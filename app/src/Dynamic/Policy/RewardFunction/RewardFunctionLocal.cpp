#include "Dynamic/Policy/RewardFunction/RewardFunctionLocal.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

Reward RewardFunctionLocal::operator()(const Env::Env &env, const Env::Vehicle &vehicle) {
    Time leftLane = env.getTime();
    assert(vehicle.enteredLane == vehicle.path.back().first);

    Time t = leftLane - vehicle.enteredLane;

    Env::Lane &lane = vehicle.position.lane;

    t *= (lane.moving.size() + lane.stopped.size());

    return -t;
}

RewardFunctionLocal RewardFunctionLocal::INSTANCE;
