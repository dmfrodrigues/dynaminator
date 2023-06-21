#pragma once

#include <functional>
#include <random>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
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

        State apply(Action action) const;

        std::vector<Action> possibleActions();
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
    double epsilon;

    // clang-format off
    mutable std::unordered_map<
        State,
        std::map<
            Action,
            Reward,
            ActionCmp
        >,
        utils::reference_wrapper::hash    <State::type>,
        utils::reference_wrapper::equal_to<State::type>
    > QMatrix;
    // clang-format on

    Reward&       Q(const State& state, const Action& action);
    const Reward& Q(const State& state, const Action& action) const;

    Reward estimateInitialValue(const State& state, const Action& action) const;

    Reward estimateOptimalFutureValue(const State& state, const Action& action) const;

    void updateMatrix(State state, Action action, Reward reward);

   public:
    QLearner(
        Env::Env&                   env,
        const SUMO::Network&        network,
        const Dynamic::SUMOAdapter& adapter,
        const Env::Edge&            destinationEdge,
        Reward                      alpha   = 0.5,
        Reward                      gamma   = 1.0,
        double                      epsilon = 0.05
    );

    void setAlpha(Reward alpha);
    void setEpsilon(double epsilon);

    void dump() const;

    class Policy: public Vehicle::Policy {
        QLearner& qLearner;

        const Env::Vehicle::ID vehicleID;

        std::mt19937& gen;

       public:
        Policy(QLearner& qLearner, Env::Vehicle::ID vehicleID, std::mt19937& gen);

        virtual Env::Lane& pickInitialLane(
            Vehicle&  vehicle,
            Env::Env& env
        ) override;

        virtual std::shared_ptr<Vehicle::Policy::Action> pickConnection(
            Env::Env& env
        ) override;

        struct Action: public Vehicle::Policy::Action {
            QLearner& qLearner;

            Action(Env::Connection& connection, Env::Lane& lane, QLearner& qLearner);

            virtual void reward(Time t) override;
        };

        struct ActionLeave: public Action {
           private:
            Env::Lane& stateLane;

           public:
            ActionLeave(Env::Lane& stateLane, QLearner& qLearner);

            virtual void reward(Time t) override;
        };

        class Factory: public Vehicle::Policy::Factory {
            Env::Env&                   env;
            const SUMO::NetworkTAZs&    sumo;
            const Dynamic::SUMOAdapter& adapter;

            std::mt19937 gen;

            std::map<Dynamic::Env::Edge::ID, Dynamic::QLearner> qLearners;

           public:
            Factory(
                Env::Env&                       env,
                const SUMO::NetworkTAZs&        sumo,
                const Dynamic::SUMOAdapter&     adapter,
                std::random_device::result_type seed = std::random_device()()
            );

            virtual std::shared_ptr<Vehicle::Policy> create(
                Vehicle::ID      id,
                Time             depart,
                const Env::Edge& from,
                const Env::Edge& to
            ) override;

            void dump() const;
        };
    };
};
}  // namespace Dynamic
