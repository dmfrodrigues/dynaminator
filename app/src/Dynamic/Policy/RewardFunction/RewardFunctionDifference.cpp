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
    {
        Length K = (double)(N) / lane.edge.length;
        Speed  v = lane.edge.calculateSpeed() * (1.0 - K / Env::Lane::K_JAM);
        v        = max(v, Env::Lane::QUEUE_SPEED);

        Length queuePos = lane.edge.length;

        tWith += queuePos / v;
    }
    Time tWithout = 0.0;
    {
        Length K = (double)(N - 1) / lane.edge.length;
        Speed  v = lane.edge.calculateSpeed() * (1.0 - K / Env::Lane::K_JAM);
        v        = max(v, Env::Lane::QUEUE_SPEED);

        Length queuePos = lane.edge.length;

        tWithout += queuePos / v;
    }
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
