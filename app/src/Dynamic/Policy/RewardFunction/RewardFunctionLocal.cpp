#include "Dynamic/Policy/RewardFunction/RewardFunctionLocal.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

Reward RewardFunctionLocal::operator()(const Env::Env &env, const Env::Vehicle &vehicle) {
    Time leftLane = env.getTime();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    assert(vehicle.enteredLane == vehicle.path.back().first);
#pragma GCC diagnostic pop

    Time t = leftLane - vehicle.enteredLane;

    Env::Lane &lane = vehicle.position.lane;

    t *= (Time)(lane.moving.size() + lane.stopped.size());

    return -t;
}

RewardFunctionLocal RewardFunctionLocal::INSTANCE;
