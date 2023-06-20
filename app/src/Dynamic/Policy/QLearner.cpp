#include "Dynamic/Policy/QLearner.hpp"

#include "Dynamic/Env/Edge.hpp"
#include "data/SUMO/Network.hpp"

using namespace std;
using namespace Dynamic;

QLearner::Action::Action(Env::Connection& connection, Env::Lane& lane):
    std::pair<std::reference_wrapper<Env::Connection>, std::reference_wrapper<Env::Lane>>(connection, lane) {}

QLearner::State::State(Env::Lane& lane):
    std::reference_wrapper<Env::Lane>(lane) {}

QLearner::State QLearner::State::apply(Action action) {
    assert(get() == action.first.get().fromLane);

    return State(action.second.get());
}

std::list<QLearner::Action> QLearner::State::possibleActions() {
    std::list<Action> actions;

    for(Env::Connection& connection: get().getOutgoingConnections()) {
        for(auto& lane: connection.toLane.edge.lanes) {
            actions.push_back(Action(connection, *lane));
        }
    }

    return actions;
}

QLearner::QLearner(
    Env::Env&                   env_,
    const SUMO::Network&        network_,
    const Dynamic::SUMOAdapter& adapter_,
    const Env::Edge&            destinationEdge_,
    Reward                      alpha_,
    Reward                      gamma_
):
    env(env_),
    network(network_),
    adapter(adapter_),
    destinationEdge(destinationEdge_),
    alpha(alpha_),
    gamma(gamma_),
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
    Env::Lane& fromLane = state.get();

    SUMO::Network::Edge::ID fromSUMOEdgeID = adapter.toSumoEdge(fromLane.edge.id);
    SUMO::Network::Edge::ID toSUMOEdgeID   = adapter.toSumoEdge(destinationEdge.id);

    const SUMO::Network::Edge& fromSUMOEdge = network.getEdge(fromSUMOEdgeID);
    const SUMO::Network::Edge& toSUMOEdge   = network.getEdge(toSUMOEdgeID);

    const SUMO::Network::Edge::Lane& fromSUMOLane = fromSUMOEdge.lanes.at(fromLane.index);
    const SUMO::Network::Edge::Lane& toSUMOLane   = toSUMOEdge.lanes.at(0);

    SUMO::Coord from = fromSUMOLane.shape.at(0);
    SUMO::Coord to   = toSUMOLane.shape.at(0);

    double d = SUMO::Coord::Distance(from, to);

    const double v = 120.0 / 3.6;

    double t = d / v;

    return -t;
}

QLearner::Reward QLearner::estimateOptimalFutureValue(State state, Action action) {
    Reward q = -1.0e9;

    State newState = state.apply(action);
    for(Action& newAction: newState.possibleActions()) {
        q = max(q, Q[newState][newAction]);
    }

    return q;
}

void QLearner::updateMatrix(State state, Action action, Reward reward) {
    Q[state][action] = (1.0 - alpha) * Q[state][action] + alpha * (reward + gamma * estimateOptimalFutureValue(state, action));
}

void QLearner::setAlpha(Reward alpha_) {
    alpha = alpha_;
}

QLearner::Policy::Policy(QLearner& qLearner_, Env::Vehicle::ID vehicleID):
    qLearner(qLearner_),
    vehicleID(vehicleID) {}

Env::Lane& QLearner::Policy::pickInitialLane(Vehicle& vehicle, Env::Env& env) {
    return *vehicle.from.lanes.at(0);
}

std::shared_ptr<Vehicle::Policy::Action> QLearner::Policy::pickConnection(Env::Env& env) {
    // TODO: implement
}

QLearner::Policy::Action::Action(Env::Connection& connection, Env::Lane& lane, QLearner& qlearner_):
    Env::Vehicle::Policy::Action(connection, lane),
    qlearner(qlearner_) {}

void QLearner::Policy::Action::reward(Time t) {
    State            s = connection.fromLane;
    QLearner::Action a = {connection, lane};
    qlearner.updateMatrix(s, a, t);
}
