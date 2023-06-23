#include "Dynamic/Policy/QLearner.hpp"

#include <limits>
#include <random>
#include <stdexcept>

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

bool QLearner::Action::operator<(const Action& other) const {
    if(connection != other.connection)
        return connection < other.connection;
    return lane < other.lane;
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
    const Env::TAZ&             destinationTAZ_,
    Reward                      alpha_,
    Reward                      gamma_,
    Reward                      xi_,
    Reward                      eta_,
    float                       epsilon_
):
    env(env_),
    network(network_),
    adapter(adapter_),
    destinationTAZ(destinationTAZ_),
    alpha(alpha_),
    gamma(gamma_),
    xi(xi_),
    eta(eta_),
    epsilon(epsilon_),
    QMatrix() {}

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
    Length d = numeric_limits<Length>::infinity();

    State sNew = s.apply(a);

    const Env::Lane& fromLane = sNew.get();

    // Destination
    const auto& sinks = destinationTAZ.sinks;
    if(sinks.find(fromLane.edge) != sinks.end())
        return 0.0;

    SUMO::Network::Edge::ID          fromSUMOEdgeID = adapter.toSumoEdge(fromLane.edge.id);
    const SUMO::Network::Edge&       fromSUMOEdge   = network.getEdge(fromSUMOEdgeID);
    const SUMO::Network::Edge::Lane& fromSUMOLane   = fromSUMOEdge.lanes.at(fromLane.index);

    SUMO::Coord from = fromSUMOLane.shape.front();

    for(const Env::Edge& destinationEdge: destinationTAZ.sinks) {
        SUMO::Network::Edge::ID    toSUMOEdgeID = adapter.toSumoEdge(destinationEdge.id);
        const SUMO::Network::Edge& toSUMOEdge   = network.getEdge(toSUMOEdgeID);

        SUMO::Coord to = toSUMOEdge.getShape().front();

        d = min(d, SUMO::Coord::Distance(from, to));
    }

    if(d >= numeric_limits<Length>::infinity())
        throw domain_error("Distance is infinite");

    const double v = 50.0 / 3.6;

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
    const Env::Lane& currentLane = s.get();
    const Env::Edge& currentEdge = currentLane.edge;

    const SUMO::Network::Edge::Lane& currentSumoLane =
        network.getEdge(adapter.toSumoEdge(currentEdge.id))
            .lanes.at(currentLane.index);

    SUMO::Coord now = currentSumoLane.getShape().front();

    Reward bestH = -numeric_limits<Reward>::infinity();
    Action bestA(Env::Connection::INVALID, Env::Lane::INVALID);

    vector<SUMO::Coord> sinksPos;
    for(const Env::Edge& e: destinationTAZ.sinks) {
        SUMO::Coord sinkPos = network.getEdge(adapter.toSumoEdge(e.id)).getShape().front();
        sinksPos.push_back(sinkPos);
    }

    for(Action& a: s.possibleActions()) {
        const SUMO::Network::Edge::Lane& nextSumoLane =
            network.getEdge(adapter.toSumoEdge(a.lane.edge.id))
                .lanes.at(a.lane.index);

        SUMO::Coord next = nextSumoLane.getShape().back();

        // Get closest sink
        SUMO::Coord destination;
        Length      dBest = numeric_limits<Length>::infinity();
        for(const SUMO::Coord& sinkPos: sinksPos) {
            Length d = SUMO::Coord::Distance(next, sinkPos);
            if(d < dBest) {
                dBest       = d;
                destination = sinkPos;
            }
        }
        assert(dBest < numeric_limits<Length>::infinity());

        // Determine angle
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
    if(xi == 0.0) return 0.0;

    Action bestA = heuristicPolicy(st);

    if(at != bestA) return 0.0;

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
    Reward& q = Q(s, a);

    q = (1.0 - alpha) * q + alpha * (reward + gamma * estimateOptimalFutureValue(s, a));
}

void QLearner::setAlpha(Reward alpha_) {
    alpha = alpha_;
}

void QLearner::setEpsilon(float epsilon_) {
    epsilon = epsilon_;
}

void QLearner::dump() const {
    stringstream ss;
    ss << "Dumping QLearner, destination TAZ is " << destinationTAZ.id << endl;
    for(const auto& [state, m]: QMatrix) {
        for(const auto& [action, q]: m) {
            ss
                << "  destTAZ " << destinationTAZ.id
                << ", a(conn: "
                << action.connection.fromLane.edge.id << "_" << action.connection.fromLane.index << " → "
                << action.connection.toLane.edge.id << "_" << action.connection.toLane.index
                << ", lane: " << action.lane.edge.id << "_" << action.lane.index
                << "), q=" << q
                << "\n";
        }
    }
    cerr << ss.rdbuf();
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
    Reward           bestQ = -numeric_limits<Reward>::infinity();
    QLearner::Action bestAction(Env::Connection::INVALID, Env::Lane::INVALID);

    for(Env::Edge& edge: vehicle.fromTAZ.sources) {
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
    }

    Env::Lane& startLane = bestAction.connection.fromLane;

    if(startLane == Env::Lane::INVALID) {
        // clang-format off
        throw logic_error(
            "Could not find suitable lane to start vehicle on; "s +
            "origin TAZ " + to_string(vehicle.fromTAZ.id) +
            ", goal TAZ " + to_string(vehicle.toTAZ.id)
        );
        // clang-format on
    }

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

    auto& sinks = vehicle.toTAZ.sinks;
    if(sinks.find(vehicle.position.lane.edge) != sinks.end()) {
        return make_shared<QLearner::Policy::ActionLeave>(vehicle.position.lane, qLearner);
    }

    State s = vehicle.position.lane;

    vector<pair<double, QLearner::Action>> actions;
    {
        vector<QLearner::Action> actionsVtr = s.possibleActions();

        actions.reserve(actions.size());

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

        // Remove impossible actions
        while(!actions.empty() && actions.back().first <= -numeric_limits<Reward>::infinity())
            actions.pop_back();
    }

    if(actions.empty())
        return make_shared<ActionLeave>(vehicle.position.lane, qLearner);

    uniform_real_distribution<float> probDistribution(0.0, 1.0);

    float p = probDistribution(gen);
    if(p < qLearner.epsilon) {
        // Pick random connection
        uniform_int_distribution<size_t> actionsDistribution(0, actions.size() - 1);

        const QLearner::Action& a = actions.at(actionsDistribution(gen)).second;

        return make_shared<QLearner::Policy::Action>(a.connection, a.lane, qLearner);
    } else {
        // Pick among the best
        vector<double> chances;
        chances.reserve(actions.size());
        for(auto it = actions.begin(); it != actions.end(); ++it) {
            double delta  = actions.front().first - it->first;
            double chance = exp(-LAMBDA * delta);
            it->first     = chance;
            chances.emplace_back(chance);
        }

        discrete_distribution<size_t> actionsDistribution(chances.begin(), chances.end());

        size_t n = actionsDistribution(gen);

        const QLearner::Action& a = actions.at(n).second;

        return make_shared<QLearner::Policy::Action>(a.connection, a.lane, qLearner);
    }
}

QLearner::Policy::Action::Action(Env::Connection& connection_, Env::Lane& lane_, QLearner& qLearner_):
    Env::Vehicle::Policy::Action(connection_, lane_),
    qLearner(qLearner_) {}

void QLearner::Policy::Action::reward(Reward r) {
    auto& sinks = qLearner.destinationTAZ.sinks;
    if(sinks.find(connection.toLane.edge) != sinks.end()) {
        return;
    }

    State            s = connection.fromLane;
    QLearner::Action a = {connection, lane};
    qLearner.updateMatrix(s, a, r);
}

QLearner::Policy::ActionLeave::ActionLeave(Env::Lane& stateLane_, QLearner& qLearner_):
    Action(Env::Connection::LEAVE, Env::Lane::INVALID, qLearner_),
    stateLane(stateLane_) {}

void QLearner::Policy::ActionLeave::reward(Reward) {
    auto& sinks = qLearner.destinationTAZ.sinks;
    if(sinks.find(stateLane.edge) == sinks.end()) {
        for(Env::Connection& conn: stateLane.edge.getIncomingConnections()) {
            State            s = conn.fromLane;
            QLearner::Action a = {conn, stateLane};

            qLearner.Q(s, a) = -numeric_limits<Reward>::infinity();
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
    Vehicle::ID     id,
    Time            depart,
    const Env::TAZ& fromTAZ,
    const Env::TAZ& toTAZ
) {
    auto it = qLearners.find(toTAZ.id);
    if(it == qLearners.end()) {
        // clang-format off
        it = qLearners.emplace(
            toTAZ.id,
            Dynamic::QLearner(
                env,
                sumo.network,
                adapter,
                toTAZ
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
