#include "Dynamic/Policy/RewardFunction/RewardFunctionGreedy.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

Reward Dynamic::RewardFunctionGreedy::operator()(const Env::Env &env, const Env::Vehicle &vehicle) {
    Time leftLane = env.getTime();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    assert(vehicle.enteredLane == vehicle.path.back().first);
#pragma GCC diagnostic pop

    Time t = leftLane - vehicle.enteredLane;

    Reward r = -t;

    return r;
}

RewardFunctionGreedy Dynamic::RewardFunctionGreedy::INSTANCE;
