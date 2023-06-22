#include "Dynamic/Policy/QLearner.hpp"

#include <limits>
#include <random>

#include "Dynamic/Env/Edge.hpp"
#include "data/SUMO/Network.hpp"
#include "utils/reference_wrapper.hpp"

using namespace std;
using namespace Dynamic;

QLearner::Action::Action(Env::Connection& connection_, Env::Lane& lane_):
    connection(connection_),
    lane(lane_) {}

QLearner::Action::Action(const Action& action):
    connection(action.connection),
    lane(action.lane) {}

QLearner::Action& QLearner::Action::operator=(const Action& other) {
    new(this) Action(other.connection, other.lane);
    return *this;
}

bool QLearner::Action::operator==(const Action& other) const {
    return connection == other.connection && lane == other.lane;
}

bool QLearner::Action::operator!=(const Action& other) const {
    return !(*this == other);
}

QLearner::State::State(Env::Lane& lane):
    reference_wrapper<Env::Lane>(lane) {}

QLearner::State QLearner::State::apply(Action action) const {
    assert(get() == action.connection.fromLane);

    return State(action.lane);
}

vector<QLearner::Action> QLearner::State::possibleActions() {
    vector<Action> actions;

    for(Env::Connection& connection: get().getOutgoingConnections()) {
        for(Env::Lane& lane: connection.toLane.edge.lanes) {
            actions.push_back(Action(connection, lane));
        }
    }

    return actions;
}

vector<QLearner::Action> QLearner::State::possibleActions() const {
    vector<Action> actions;

    for(Env::Connection& connection: get().getOutgoingConnections()) {
        for(Env::Lane& lane: connection.toLane.edge.lanes) {
            actions.push_back(Action(connection, lane));
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
    Reward                      xi_,
    Reward                      eta_,
    double                      epsilon_
):
    env(env_),
    network(network_),
    adapter(adapter_),
    destinationEdge(destinationEdge_),
    alpha(alpha_),
    gamma(gamma_),
    xi(xi_),
    eta(eta_),
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

QLearner::Reward& QLearner::Q(const State& s, const Action& a) {
    auto& q = QMatrix[s];

    auto it = q.find(a);
    if(it != q.end()) return it->second;

    return q[a] = estimateInitialValue(s, a);
}

const QLearner::Reward& QLearner::Q(const State& s, const Action& a) const {
    auto& q = QMatrix[s];

    auto it = q.find(a);
    if(it != q.end()) return it->second;

    return q[a] = estimateInitialValue(s, a);
}

QLearner::Reward QLearner::estimateInitialValue(const State& s, const Action& a) const {
    State sNew = s.apply(a);

    const Env::Lane& fromLane = sNew.get();

    SUMO::Network::Edge::ID fromSUMOEdgeID = adapter.toSumoEdge(fromLane.edge.id);
    SUMO::Network::Edge::ID toSUMOEdgeID   = adapter.toSumoEdge(destinationEdge.id);

    const SUMO::Network::Edge& fromSUMOEdge = network.getEdge(fromSUMOEdgeID);
    const SUMO::Network::Edge& toSUMOEdge   = network.getEdge(toSUMOEdgeID);

    const SUMO::Network::Edge::Lane& fromSUMOLane = fromSUMOEdge.lanes.at(fromLane.index);

    SUMO::Coord from = fromSUMOLane.shape.front();
    SUMO::Coord to   = toSUMOEdge.getShape().front();

    double d = SUMO::Coord::Distance(from, to);

    const double v = 10.0 / 3.6;

    double t = d / v;

    return -t;
}

QLearner::Reward QLearner::estimateOptimalValue(const State& s) const {
    Reward q = -numeric_limits<Reward>::infinity();

    for(const Action& a: s.possibleActions()) {
        q = max(q, Q(s, a));
    }

    return q;
}

QLearner::Reward QLearner::estimateOptimalFutureValue(const State& s, const Action& a) const {
    State sNew = s.apply(a);

    return estimateOptimalValue(sNew);
}

QLearner::Action QLearner::heuristicPolicy(const State& s) const {
    const Env::Edge& currentEdge = s.get().edge;

    SUMO::Coord now         = network.getEdge(adapter.toSumoEdge(currentEdge.id)).getShape().front();
    SUMO::Coord destination = network.getEdge(adapter.toSumoEdge(destinationEdge.id)).getShape().front();

    Reward bestH = -numeric_limits<Reward>::infinity();
    Action bestA(Env::Connection::INVALID, Env::Lane::INVALID);

    for(Action& a: s.possibleActions()) {
        SUMO::Coord next = network.getEdge(adapter.toSumoEdge(a.lane.edge.id)).getShape().back();

        Vector2 v1 = next - now;
        Vector2 v2 = destination - now;

        double theta = Vector2::Angle(v1, v2);

        Reward h = -theta;

        if(h > bestH) {
            bestH = h;
            bestA = a;
        }
    }

    assert(bestA.connection != Env::Connection::INVALID);

    return bestA;
}

QLearner::Reward QLearner::heuristic(const State& st, const Action& at) const {
    Action bestA = heuristicPolicy(st);

    if(at != bestA) return 0;

    Reward q = Q(st, at);

    if(q == -numeric_limits<Reward>::infinity())
        return 0;

    Reward H = estimateOptimalValue(st) - q + eta;

    // assert(!isnan(H));
    // assert(H < numeric_limits<Reward>::infinity());
    // assert(H > -numeric_limits<Reward>::infinity());

    Reward h = xi * H;

    return h;
}

void QLearner::updateMatrix(const State& s, const Action& a, Reward reward) {
    Q(s, a) = (1.0 - alpha) * Q(s, a) + alpha * (reward + gamma * estimateOptimalFutureValue(s, a));
}

void QLearner::setAlpha(Reward alpha_) {
    alpha = alpha_;
}

void QLearner::setEpsilon(double epsilon_) {
    epsilon = epsilon_;
}

void QLearner::dump() const {
    cerr << "Dumping QLearner, destination edge is " << destinationEdge.id << endl;
    for(const auto& [state, m]: QMatrix) {
        for(const auto& [action, q]: m) {
            cerr
                << "    Destination edge " << destinationEdge.id
                << ", action(connection: "
                << action.connection.fromLane.edge.id << "_" << action.connection.fromLane.index << " → "
                << action.connection.toLane.edge.id << "_" << action.connection.toLane.index
                << ", lane: " << action.lane.edge.id << "_" << action.lane.index
                << "), q=" << q
                << "\n";
        }
    }
}

QLearner::Policy::Policy(
    QLearner&        qLearner_,
    Env::Vehicle::ID vehicleID,
    mt19937&         gen_
):
    qLearner(qLearner_),
    vehicleID(vehicleID),
    gen(gen_) {}

Env::Lane& QLearner::Policy::pickInitialLane(Vehicle& vehicle, Env::Env& env) {
    Env::Edge& edge = vehicle.from;

    Reward           bestQ = -numeric_limits<Reward>::infinity();
    QLearner::Action bestAction(Env::Connection::INVALID, Env::Lane::INVALID);

    for(Env::Connection& connection: edge.getOutgoingConnections()) {
        State s = connection.fromLane;
        for(Env::Lane& lane: connection.toLane.edge.lanes) {
            QLearner::Action a = {connection, lane};

            Reward q = qLearner.Q(s, a);

            if(q > bestQ) {
                bestQ      = q;
                bestAction = a;
            }
        }
    }

    Env::Lane& startLane = bestAction.connection.fromLane;

    if(startLane == Env::Lane::INVALID) {
        throw logic_error("Could not find suitable lane to start vehicle on; edge is " + to_string(edge.id) + ", goal is edge " + to_string(qLearner.destinationEdge.id));
    }

    // if(bestQ < -86400) {
    //     // clang-format off
    //     // throw logic_error(
    //         cerr <<
    //         "Best edge has bestQ=" + to_string(bestQ) +
    //         ", which is suspicious; chosen lane is " + to_string(startLane.edge.id) + "_" + to_string(startLane.index) +
    //         ", goal is edge " + to_string(qLearner.destinationEdge.id)
    //         << endl;
    //     // );
    //     // clang-format on
    // }

    return startLane;
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

shared_ptr<Vehicle::Policy::Action> QLearner::Policy::pickConnection(Env::Env& env) {
    Env::Vehicle& vehicle = env.getVehicle(vehicleID);

    if(vehicle.position.lane.edge == vehicle.to) {
        return make_shared<QLearner::Policy::ActionLeave>(vehicle.position.lane, qLearner);
    }

    State s = vehicle.position.lane;

    vector<pair<double, QLearner::Action>> actions;
    {
        vector<QLearner::Action> actionsVtr = s.possibleActions();

        for(const QLearner::Action& a: actionsVtr) {
            Reward q = qLearner.Q(s, a);
            q += qLearner.heuristic(s, a);

            if(q <= -numeric_limits<Reward>::infinity())
                continue;

            actions.emplace_back(q, a);
        }

        sort(actions.begin(), actions.end(), [](const auto& a, const auto& b) -> bool {
            return a.first > b.first;
        });
    }

    if(actions.empty())
        return make_shared<ActionLeave>(vehicle.position.lane, qLearner);

    uniform_real_distribution<> probDistribution(0.0, 1.0);

    double p = probDistribution(gen);
    if(p < qLearner.epsilon) {
        // Pick random connection
        uniform_int_distribution<> actionsDistribution(0, actions.size() - 1);

        QLearner::Action a = actions.at(actionsDistribution(gen)).second;

        return make_shared<QLearner::Policy::Action>(a.connection, a.lane, qLearner);
    } else {
        // Pick among the best
        vector<double> chances;
        for(auto it = actions.begin(); it != actions.end(); ++it) {
            double delta  = actions.front().first - it->first;
            double chance = exp(-LAMBDA * delta);
            it->first     = chance;
            chances.push_back(chance);
        }

        discrete_distribution<size_t> actionsDistribution(chances.begin(), chances.end());

        size_t n = actionsDistribution(gen);

        QLearner::Action a = actions.at(n).second;

        return make_shared<QLearner::Policy::Action>(a.connection, a.lane, qLearner);
    }
}

QLearner::Policy::Action::Action(Env::Connection& connection_, Env::Lane& lane_, QLearner& qLearner_):
    Env::Vehicle::Policy::Action(connection_, lane_),
    qLearner(qLearner_) {}

void QLearner::Policy::Action::reward(Reward r) {
    State            s = connection.fromLane;
    QLearner::Action a = {connection, lane};
    qLearner.updateMatrix(s, a, r);
}

QLearner::Policy::ActionLeave::ActionLeave(Env::Lane& stateLane_, QLearner& qLearner_):
    Action(Env::Connection::LEAVE, Env::Lane::INVALID, qLearner_),
    stateLane(stateLane_) {}

void QLearner::Policy::ActionLeave::reward(Reward) {
    if(stateLane.edge != qLearner.destinationEdge) {
        for(Env::Connection& conn: stateLane.edge.getIncomingConnections()) {
            State            s = conn.fromLane;
            QLearner::Action a = {conn, stateLane};

            // clang-format off
            qLearner.Q(s, a) = (
                stateLane.edge == qLearner.destinationEdge ?
                0 :
                -numeric_limits<Reward>::infinity()
            );
            // clang-format on
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

void QLearner::Policy::Factory::dump() const {
    for(const auto& pr: qLearners) {
        const QLearner& qLearner = pr.second;
        qLearner.dump();
    }
}
