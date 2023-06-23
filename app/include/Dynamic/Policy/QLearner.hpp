#pragma once

#include <functional>
#include <random>

#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "utils/reference_wrapper.hpp"

namespace Dynamic {
class QLearner {
    typedef Vehicle::Policy::Reward Reward;

   public:
    // clang-format off
    class Action {
        // clang-format on
       public:
        Env::Connection& connection;
        Env::Lane&       lane;

        Action(Env::Connection& connection, Env::Lane& lane);
        Action(const Action& action);

        Action& operator=(const Action& other);

        bool operator==(const Action& other) const;
        bool operator!=(const Action& other) const;

        bool operator<(const Action& other) const;
    };

    class State: public std::reference_wrapper<Env::Lane> {
       public:
        State(Env::Lane& lane);

        State apply(Action action) const;

        std::vector<Action> possibleActions();
        std::vector<Action> possibleActions() const;
    };

   private:
    Env::Env&                   env;
    const SUMO::Network&        network;
    const Dynamic::SUMOAdapter& adapter;
    const Env::TAZ&             destinationTAZ;

    Alg::ShortestPath::Dijkstra sp;

    Reward alpha, gamma, xi, eta;
    float  epsilon;

    // clang-format off
    mutable std::unordered_map<
        State,
        std::map<
            Action,
            Reward
        >,
        utils::reference_wrapper::hash    <State::type>,
        utils::reference_wrapper::equal_to<State::type>
    > QMatrix;
    // clang-format on

    Reward&       Q(const State& s, const Action& a);
    const Reward& Q(const State& s, const Action& a) const;

    Reward estimateInitialValue(const State& s, const Action& a) const;

    Reward estimateOptimalValue(const State& s) const;
    Reward estimateOptimalFutureValue(const State& s, const Action& a) const;

    Action heuristicPolicy(const State& s) const;
    Reward heuristic(const State& s, const Action& a) const;

    void updateMatrix(const State& s, const Action& a, Reward reward);

   public:
    /**
     * @brief Construct a new Q-learner.
     *
     * @param env
     * @param network
     * @param adapter
     * @param destinationTAZ
     * @param alpha
     * @param gamma
     * @param xi
     * @param eta
     * @param epsilon
     */
    QLearner(
        Env::Env&                   env,
        const SUMO::Network&        network,
        const Dynamic::SUMOAdapter& adapter,
        const Env::TAZ&             destinationTAZ,
        Reward                      alpha   = 0.5,
        Reward                      gamma   = 1.0,
        Reward                      xi      = 0.0,
        Reward                      eta     = 1.0,
        float                       epsilon = 0.1f
    );

    void setAlpha(Reward alpha);
    void setEpsilon(float epsilon);

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

            virtual void reward(Reward r) override;
        };

        struct ActionLeave: public Action {
           private:
            Env::Lane& stateLane;

           public:
            ActionLeave(Env::Lane& stateLane, QLearner& qLearner);

            virtual void reward(Reward r) override;
        };

        class Factory: public Vehicle::Policy::Factory {
            Env::Env&                   env;
            const SUMO::NetworkTAZs&    sumo;
            const Dynamic::SUMOAdapter& adapter;

            std::mt19937 gen;

           public:
            std::map<Dynamic::Env::Edge::ID, Dynamic::QLearner> qLearners;

           public:
            Factory(
                Env::Env&                       env,
                const SUMO::NetworkTAZs&        sumo,
                const Dynamic::SUMOAdapter&     adapter,
                std::random_device::result_type seed = 0
            );

            virtual std::shared_ptr<Vehicle::Policy> create(
                Vehicle::ID     id,
                Time            depart,
                const Env::TAZ& fromTAZ,
                const Env::TAZ& toTAZ
            ) override;

            void dump() const;
        };
    };
};
}  // namespace Dynamic
