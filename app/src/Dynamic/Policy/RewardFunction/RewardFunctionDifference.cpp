#include "Dynamic/Policy/RewardFunction/RewardFunctionDifference.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

Reward RewardFunctionDifference::operator()(const Env::Env &env, const Env::Vehicle &vehicle) {
    const Env::Lane &lane = vehicle.position.lane;

    Time leftLane = env.getTime();
    assert(vehicle.enteredLane == vehicle.path.back().first);

    Time t = leftLane - vehicle.enteredLane;

    Reward r = -t;

    const size_t N = lane.moving.size() + lane.stopped.size();

    Time tWith = 0.0;
    // {
    Length KWith = (double)(N) / lane.edge.length;
    Speed  vWith = lane.edge.calculateSpeed() * (1.0 - KWith / Env::Lane::K_JAM);
    vWith        = max(vWith, Env::Lane::QUEUE_SPEED);

    Length queuePosWith = min(lane.queuePosition(), lane.edge.length);

    tWith += queuePosWith / vWith;
    // }
    Time tWithout = 0.0;
    // {
    Length KWithout = (double)(N - 1) / lane.edge.length;
    Speed  vWithout = lane.edge.calculateSpeed() * (1.0 - KWithout / Env::Lane::K_JAM);
    vWithout        = max(vWithout, Env::Lane::QUEUE_SPEED);

    Length queuePosWithout = min(lane.queuePosition(), lane.edge.length);

    tWithout += queuePosWithout / vWithout;
    // }
    Time deltaMoving = tWith - tWithout;

    // Time deltaQueue = (lane.stopped.empty() ? 0 : Env::Lane::JUNCTION_PERIOD);
    // Time deltaQueue = Env::Lane::JUNCTION_PERIOD;
    Time deltaQueue = 0.0;

    if(N >= 1) {
        r -= 0.5 * (deltaMoving + deltaQueue) * (N - 1);
        // if(N >= 30 && deltaMoving > 0)
        //     cerr
        //         << ", KWith = " << KWith
        //         << ", KWithout = " << KWithout
        //         << ", vWith = " << vWith
        //         << ", vWithout = " << vWithout
        //         << ", tWith = " << tWith
        //         << ", tWithout = " << tWithout
        //         << ", deltaMoving = " << deltaMoving
        //         << ", deltaQueue = " << deltaQueue
        //         << ", N = " << N
        //         << ", r = " << r
        //         << endl;
    }

    return r;
}

RewardFunctionDifference RewardFunctionDifference::INSTANCE;
