#include "Dynamic/Policy/QLearner.hpp"

#include <random>

#include "Dynamic/Env/Edge.hpp"
#include "data/SUMO/Network.hpp"

using namespace std;
using namespace Dynamic;

QLearner::Action::Action(Env::Connection& connection, Env::Lane& lane):
    std::pair<std::reference_wrapper<Env::Connection>, std::reference_wrapper<Env::Lane>>(connection, lane) {}

QLearner::State::State(Env::Lane& lane):
    std::reference_wrapper<Env::Lane>(lane) {}

QLearner::State QLearner::State::apply(Action action) const {
    assert(get() == action.first.get().fromLane);

    return State(action.second.get());
}

std::vector<QLearner::Action> QLearner::State::possibleActions() {
    std::vector<Action> actions;

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
    Reward                      gamma_,
    double                      epsilon_
):
    env(env_),
    network(network_),
    adapter(adapter_),
    destinationEdge(destinationEdge_),
    alpha(alpha_),
    gamma(gamma_),
    epsilon(epsilon_),
    QMatrix() {
    // for(Env::Edge& edge: env.getEdges()) {
    //     for(auto& fromLane: edge.lanes) {
    //         for(Env::Connection& connection: fromLane->getOutgoingConnections()) {
    //             for(auto& toLane: connection.toLane.edge.lanes) {
    //                 State  s = *fromLane;
    //                 Action a = {connection, *toLane};
    //                 Reward q = estimateInitialValue(s, a);

    //                 Q(s, a) = q;
    //             }
    //         }
    //     }
    // }
}

QLearner::Reward& QLearner::Q(const State& state, const Action& action) {
    auto& q = QMatrix[state];

    auto it = q.find(action);
    if(it != q.end()) return it->second;

    return q[action] = estimateInitialValue(state, action);
}

const QLearner::Reward& QLearner::Q(const State& state, const Action& action) const {
    auto& q = QMatrix[state];

    auto it = q.find(action);
    if(it != q.end()) return it->second;

    return q[action] = estimateInitialValue(state, action);
}

QLearner::Reward QLearner::estimateInitialValue(const State& state, const Action& action) const {
    const Env::Lane& fromLane = state.get();

    SUMO::Network::Edge::ID fromSUMOEdgeID = adapter.toSumoEdge(fromLane.edge.id);
    SUMO::Network::Edge::ID toSUMOEdgeID   = adapter.toSumoEdge(destinationEdge.id);

    const SUMO::Network::Edge& fromSUMOEdge = network.getEdge(fromSUMOEdgeID);
    const SUMO::Network::Edge& toSUMOEdge   = network.getEdge(toSUMOEdgeID);

    const SUMO::Network::Edge::Lane& fromSUMOLane = fromSUMOEdge.lanes.at(fromLane.index);
    const SUMO::Network::Edge::Lane& toSUMOLane   = toSUMOEdge.lanes.at(0);

    SUMO::Coord from = fromSUMOLane.shape.at(0);
    SUMO::Coord to   = toSUMOLane.shape.at(0);

    double d = SUMO::Coord::Distance(from, to);

    const double v = 10.0 / 3.6;

    double t = d / v;

    return -t;
}

QLearner::Reward QLearner::estimateOptimalFutureValue(const State& state, const Action& action) const {
    Reward q = -1.0e9;

    State newState = state.apply(action);
    for(Action& newAction: newState.possibleActions()) {
        q = max(q, Q(newState, newAction));
    }

    return q;
}

void QLearner::updateMatrix(State state, Action action, Reward reward) {
    QMatrix[state][action] = (1.0 - alpha) * QMatrix[state][action] + alpha * (reward + gamma * estimateOptimalFutureValue(state, action));
}

void QLearner::setAlpha(Reward alpha_) {
    alpha = alpha_;
}

void QLearner::setEpsilon(double epsilon_) {
    epsilon = epsilon_;
}

QLearner::Policy::Policy(
    QLearner&        qLearner_,
    Env::Vehicle::ID vehicleID,
    std::mt19937&    gen_
):
    qLearner(qLearner_),
    vehicleID(vehicleID),
    gen(gen_) {}

Env::Lane& QLearner::Policy::pickInitialLane(Vehicle& vehicle, Env::Env& env) {
    return *vehicle.from.lanes.at(0);
}

/**
 * @brief Coefficient of the exponential distribution used to pick among the
 * best actions.
 *
 * The smaller LAMBDA is, the more likely it is to select an action other than
 * the best.
 *
 * LAMBDA = ln(2)/x
 *
 * This equation means that, for the chance of an action with penalty x to be 0.5, LAMBDA needs to conform to this equation.
 *
 * The current value LAMBDA = 0.1 means the x that gives a 50% chance is 6.931.
 */
const double LAMBDA = 100;

std::shared_ptr<Vehicle::Policy::Action> QLearner::Policy::pickConnection(Env::Env& env) {
    Env::Vehicle& vehicle = env.getVehicle(vehicleID);

    if(vehicle.position.lane.edge == vehicle.to) {
        return make_shared<QLearner::Policy::ActionLeave>(vehicle.position.lane, qLearner);
    }

    State s = vehicle.position.lane;

    vector<QLearner::Action> actions = s.possibleActions();

    if(actions.empty())
        return make_shared<ActionLeave>(vehicle.position.lane, qLearner);

    std::uniform_real_distribution<> probDistribution(0.0, 1.0);

    double p = probDistribution(gen);
    if(p < qLearner.epsilon) {
        // Pick random connection
        std::uniform_int_distribution<> actionsDistribution(0, actions.size() - 1);

        QLearner::Action a = actions.at(actionsDistribution(gen));

        return make_shared<QLearner::Policy::Action>(a.first, a.second, qLearner);
    } else {
        // Pick among the best
        vector<pair<double, QLearner::Action>> qs;
        for(const QLearner::Action& a: actions) {
            Reward q = qLearner.Q(s, a);
            qs.emplace_back(q, a);
        }
        sort(qs.begin(), qs.end(), [](const auto& a, const auto& b) -> bool {
            return a.first > b.first;
        });
        for(auto it = qs.rbegin(); it != qs.rend(); ++it) {
            double delta  = qs.front().first - it->first;
            double chance = exp(-LAMBDA * delta);
            it->first     = chance;
        }

        vector<double> chances;
        for(const auto& pr: qs)
            chances.emplace_back(pr.first);

        discrete_distribution<size_t> actionsDistribution{chances.begin(), chances.end()};

        QLearner::Action a = qs.at(actionsDistribution(gen)).second;

        return make_shared<QLearner::Policy::Action>(a.first, a.second, qLearner);
    }
}

QLearner::Policy::Action::Action(Env::Connection& connection, Env::Lane& lane, QLearner& qlearner_):
    Env::Vehicle::Policy::Action(connection, lane),
    qlearner(qlearner_) {}

void QLearner::Policy::Action::reward(Time t) {
    State            s = connection.fromLane;
    QLearner::Action a = {connection, lane};
    qlearner.updateMatrix(s, a, t);
}

QLearner::Policy::ActionLeave::ActionLeave(Env::Lane& stateLane_, QLearner& qLearner_):
    Action(Env::Connection::LEAVE, Env::Lane::INVALID, qLearner_),
    stateLane(stateLane_) {}

void QLearner::Policy::ActionLeave::reward(Time t) {
    if(stateLane.edge != qlearner.destinationEdge) {
        for(Env::Connection& connection: stateLane.edge.getIncomingConnections()) {
            State            s = connection.fromLane;
            QLearner::Action a = {connection, stateLane};

            qlearner.Q(s, a) = -1.0e9;
            // if(connection.fromLane.edge.id == 8429) {
            //     cerr << "Correcting action ending at "
            // }
        }
    }
}

QLearner::Policy::Factory::Factory(
    Env::Env&                   env_,
    const SUMO::NetworkTAZs&    sumo_,
    const Dynamic::SUMOAdapter& adapter_,
    random_device::result_type  seed
):
    env(env_),
    sumo(sumo_),
    adapter(adapter_),
    gen(seed) {}

shared_ptr<Vehicle::Policy> QLearner::Policy::Factory::create(
    Vehicle::ID      id,
    Time             depart,
    const Env::Edge& from,
    const Env::Edge& to
) {
    auto it = qLearners.find(to.id);
    if(it == qLearners.end()) {
        // clang-format off
        it = qLearners.emplace(
            to.id,
            Dynamic::QLearner(
                env,
                sumo.network,
                adapter,
                to
            )
        ).first;
        // clang-format on
    }

    QLearner& qLearner = it->second;

    return make_shared<QLearner::Policy>(qLearner, id, gen);
}
