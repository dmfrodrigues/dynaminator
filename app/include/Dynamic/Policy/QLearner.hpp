#pragma once

#include <functional>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "utils/reference_wrapper.hpp"

namespace Dynamic {
class QLearner {
   public:
    typedef Time Reward;
    // clang-format off
    class Action: public std::pair<
        std::reference_wrapper<Env::Connection>,
        std::reference_wrapper<Env::Lane>
    > {
        // clang-format on
       public:
        Action(Env::Connection& connection, Env::Lane& lane);
    };

    class State: public std::reference_wrapper<Env::Lane> {
       public:
        State(Env::Lane& lane);

        State apply(Action action);

        std::list<Action> possibleActions();
    };

   private:
    struct ActionCmp {
        bool operator()(const Action& a, const Action& b) const {
            if(a.first.get() != b.first.get()) return a.first.get() < b.first.get();
            return a.second.get() < b.second.get();
        }
    };

    Env::Env&                   env;
    const SUMO::Network&        network;
    const Dynamic::SUMOAdapter& adapter;
    const Env::Edge&            destinationEdge;

    Reward alpha, gamma;

    // clang-format off
    std::unordered_map<
        State,
        std::map<
            Action,
            Reward,
            ActionCmp
        >,
        utils::reference_wrapper::hash    <State::type>,
        utils::reference_wrapper::equal_to<State::type>
    > Q;
    // clang-format on

    Reward estimateInitialValue(State state, Action action);

    Reward estimateOptimalFutureValue(State state, Action action);

    void updateMatrix(State state, Action action, Reward reward);

   public:
    QLearner(
        Env::Env&                   env,
        const SUMO::Network&        network,
        const Dynamic::SUMOAdapter& adapter,
        const Env::Edge&            destinationEdge,
        Reward                      alpha = 0.1,
        Reward                      gamma = 0.5
    );

    void setAlpha(Reward alpha);

    class Policy: public Vehicle::Policy {
        QLearner& qLearner;

        const Env::Vehicle::ID vehicleID;

       public:
        Policy(QLearner& qLearner, Env::Vehicle::ID vehicleID);

        virtual Env::Lane& pickInitialLane(
            Vehicle&  vehicle,
            Env::Env& env
        ) override;

        virtual std::shared_ptr<Vehicle::Policy::Action> pickConnection(
            Env::Env& env
        ) override;

        virtual void feedback(
            const Env::Edge& e,
            Time             t
        ) override;
    };
};
}  // namespace Dynamic
