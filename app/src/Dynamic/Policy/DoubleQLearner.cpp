#include "Dynamic/Policy/DoubleQLearner.hpp"

#include <random>

#include "Dynamic/Policy/Policy.hpp"
#include "Dynamic/Policy/QLearner.hpp"

using namespace std;
using namespace Dynamic;

DoubleQLearner::PolicyFactory::PolicyFactory(
    Env::Env&                                     env_,
    const SUMO::NetworkTAZs&                      sumo_,
    const Dynamic::SUMOAdapter&                   adapter_,
    random_device::result_type                    seed,
    optional<reference_wrapper<QLearner::Logger>> policyLogger_
):
    QLearner::Policy::Factory(
        env_,
        sumo_,
        adapter_,
        seed,
        policyLogger_
    ) {}

shared_ptr<Dynamic::Policy> DoubleQLearner::PolicyFactory::create(
    Vehicle::ID id,
    Time,
    const Env::TAZ&,
    const Env::TAZ& toTAZ
) {
    auto it = qLearners.find(toTAZ.id);
    if(it == qLearners.end()) {
        // clang-format off
        it = qLearners.emplace(
            toTAZ.id,
            DoubleQLearner(
                env,
                sumo.network,
                adapter,
                toTAZ,
                gen,
                policyLogger
            )
        ).first;
        // clang-format on
    }

    QLearner& qLearner = it->second;

    return make_shared<QLearner::Policy>(qLearner, id, gen);
}

DoubleQLearner::DoubleQLearner(
    Env::Env&                                     env_,
    const SUMO::Network&                          network_,
    const Dynamic::SUMOAdapter&                   adapter_,
    const Env::TAZ&                               destinationTAZ_,
    mt19937&                                      gen_,
    optional<reference_wrapper<QLearner::Logger>> policyLogger_,
    Reward                                        alpha_,
    Reward                                        gamma_,
    Reward                                        xi_,
    Reward                                        eta_,
    float                                         epsilon_
):
    QLearner(
        env_,
        network_,
        adapter_,
        destinationTAZ_,
        policyLogger_,
        alpha_,
        gamma_,
        xi_,
        eta_,
        epsilon_
    ),
    gen(gen_) {
}

QLearner::Reward DoubleQLearner::estimateOptimalValue(const State&) const {
    throw logic_error(__PRETTY_FUNCTION__ + string(": not implemented"));
}

QLearner::Reward DoubleQLearner::estimateOptimalFutureValue(const State&, const Action&) const {
    throw logic_error(__PRETTY_FUNCTION__ + string(": not implemented"));
}

QLearner::Reward& DoubleQLearner::Q(const State&, const Action&) {
    throw logic_error(__PRETTY_FUNCTION__ + string(": not implemented"));
}

QLearner::Reward DoubleQLearner::Q(const State& s, const Action& a) const {
    return (QA(s, a) + QB(s, a)) / 2.0;
}

void DoubleQLearner::updateMatrix(const State& s, const Action& a, Reward r) {
    uniform_real_distribution<double> distribution(0.0, 1.0);

    const double x = distribution(gen);
    if(x < 0.5) {
        Reward& qA = QA(s, a);

        Reward qPrev = qA;
        Reward fB    = estimateOptimalFutureValueA(s, a);

        Reward qNew = (r + gamma * fB);

        qA += alpha * (qNew - qA);

        if(policyLogger.has_value()) {
            auto& logger = policyLogger.value().get();
            auto &D = logger.D, &DA = logger.DA;

            const double Delta = qA - qPrev;
            D += Logger::ALPHA_D * (Delta - D);
            DA += Logger::ALPHA_D * (abs(Delta) - DA);
        }
    } else {
        Reward& qB = Q(s, a);

        Reward qPrev = qB;
        Reward fA    = estimateOptimalFutureValueB(s, a);

        Reward qNew = (r + gamma * fA);

        qB += alpha * (qNew - qB);

        if(policyLogger.has_value()) {
            auto& logger = policyLogger.value().get();
            auto &D = logger.D, &DA = logger.DA;

            const double Delta = qB - qPrev;
            D += Logger::ALPHA_D * (Delta - D);
            DA += Logger::ALPHA_D * (abs(Delta) - DA);
        }
    }
}

QLearner::Reward& DoubleQLearner::QA(const State& s, const Action& a) {
    auto& q = QMatrixA[s];

    auto it = q.find(a);
    if(it != q.end()) return it->second;

    return q[a] = estimateInitialValue(s, a);
}

QLearner::Reward DoubleQLearner::QA(const State& s, const Action& a) const {
    auto it = QMatrixA.find(s);
    if(it == QMatrixA.end()) return 0.0;

    auto it2 = it->second.find(a);
    if(it2 == it->second.end()) return 0.0;

    return it2->second;
}

QLearner::Reward& DoubleQLearner::QB(const State& s, const Action& a) {
    auto& q = QMatrixB[s];

    auto it = q.find(a);
    if(it != q.end()) return it->second;

    return q[a] = estimateInitialValue(s, a);
}

QLearner::Reward DoubleQLearner::QB(const State& s, const Action& a) const {
    auto it = QMatrixB.find(s);
    if(it == QMatrixB.end()) return 0.0;

    auto it2 = it->second.find(a);
    if(it2 == it->second.end()) return 0.0;

    return it2->second;
}

QLearner::Reward DoubleQLearner::estimateOptimalValueA(const State& s) const {
    vector<Action> actions = s.possibleActions();

    vector<pair<Reward, Action>> actionRewards;
    for(const Action& a: actions) {
        actionRewards.emplace_back(QB(s, a), a);
    }

    auto [_, a] = *max_element(
        actionRewards.begin(),
        actionRewards.end()
    );

    return QA(s, a);
}

QLearner::Reward DoubleQLearner::estimateOptimalValueB(const State& s) const {
    vector<Action> actions = s.possibleActions();

    vector<pair<Reward, Action>> actionRewards;
    for(const Action& a: actions) {
        actionRewards.emplace_back(QA(s, a), a);
    }

    auto [_, a] = *max_element(
        actionRewards.begin(),
        actionRewards.end()
    );

    return QB(s, a);
}

QLearner::Reward DoubleQLearner::estimateOptimalFutureValueA(const State& s, const Action& a) const {
    State sNew = s.apply(a);

    return estimateOptimalValueA(sNew);
}

QLearner::Reward DoubleQLearner::estimateOptimalFutureValueB(const State& s, const Action& a) const {
    State sNew = s.apply(a);

    return estimateOptimalValueB(sNew);
}
