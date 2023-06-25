#include "Dynamic/Policy/RewardFunction/RewardFunctionGreedy.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

Reward Dynamic::RewardFunctionGreedy::operator()(const Env::Env &env, const Env::Vehicle &vehicle) {
    Time leftLane = env.getTime();
    assert(vehicle.enteredLane == vehicle.path.back().first);

    Time t = leftLane - vehicle.enteredLane;

    Reward r = -t;

    return r;
}

RewardFunctionGreedy Dynamic::RewardFunctionGreedy::INSTANCE;
