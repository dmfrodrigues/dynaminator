#include "Dynamic/Policy/QLearner.hpp"

using namespace std;
using namespace Dynamic;

QLearner::QLearner(Reward alpha, Reward gamma, Env::Env& env):
    alpha(alpha),
    gamma(gamma),
    Q() {
    for(Env::Edge& edge: env.getEdges()) {
        for(auto& fromLane: edge.lanes) {
            for(Env::Connection& connection: fromLane->getOutgoingConnections()) {
                for(auto& toLane: connection.toLane.edge.lanes) {
                    State  s = *fromLane;
                    Action a = {connection, *toLane};
                    Reward q = estimateInitialValue(s, a);

                    Q[s][a] = q;
                }
            }
        }
    }
}

QLearner::Reward QLearner::estimateInitialValue(State state, Action action) {
    return -1e9;
}

void QLearner::updateMatrix(State state, Action action, Reward reward) {
    // Q[state][action] = (1 - alpha) * Q[state][action] + alpha * reward;
}

void QLearner::setAlpha(Reward alpha_) {
    alpha = alpha_;
}
