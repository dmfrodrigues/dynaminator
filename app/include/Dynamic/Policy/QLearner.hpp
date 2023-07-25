#pragma once

#include <functional>
#include <random>

#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Policy/Policy.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Log/ProgressLogger.hpp"
#include "data/SUMO/Network.hpp"
#include "utils/reference_wrapper.hpp"

namespace Dynamic {

class QLearner {
   protected:
    typedef Env::Action::Reward Reward;

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

    class Logger: public Policy::Logger {
        friend class QLearner;
        friend class DoubleQLearner;

        static const double ALPHA_D;

        Reward alpha;

        double D = 0.0, DA = 0.0;

       public:
        Logger(Reward alpha);
        virtual void header(Log::ProgressLogger& logger);
        virtual void log(Log::ProgressLogger& logger);
        void         setAlpha(Reward alpha);
    };

    class Policy: public Dynamic::Policy {
        QLearner& qLearner;

        const Env::Vehicle::ID vehicleID;

        std::mt19937& gen;

       public:
        Policy(QLearner& qLearner, Env::Vehicle::ID vehicleID, std::mt19937& gen);

        virtual Env::Lane& pickInitialLane(
            Vehicle&  vehicle,
            Env::Env& env
        ) override;

        virtual std::shared_ptr<Env::Action> pickConnection(
            Env::Env& env
        ) override;

        struct Action: public Env::Action {
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

        class Factory: public Dynamic::Policy::Factory {
           protected:
            Env::Env&                   env;
            const SUMO::NetworkTAZs&    sumo;
            const Dynamic::SUMOAdapter& adapter;

            std::mt19937 gen;

            std::map<Dynamic::Env::Edge::ID, Dynamic::QLearner> qLearners;

            std::optional<std::reference_wrapper<QLearner::Logger>> policyLogger;

           public:
            Factory(
                Env::Env&                                               env,
                const SUMO::NetworkTAZs&                                sumo,
                const Dynamic::SUMOAdapter&                             adapter,
                std::random_device::result_type                         seed         = 0,
                std::optional<std::reference_wrapper<QLearner::Logger>> policyLogger = std::nullopt
            );

            virtual std::shared_ptr<Dynamic::Policy> create(
                Vehicle::ID     id,
                Time            depart,
                const Env::TAZ& fromTAZ,
                const Env::TAZ& toTAZ
            ) override;

            void dump() const;
        };
    };

   private:
    Env::Env&                   env;
    const SUMO::Network&        network;
    const Dynamic::SUMOAdapter& adapter;
    const Env::TAZ&             destinationTAZ;

   public:
    Alg::ShortestPath::Dijkstra sp;

   protected:
    Reward alpha, gamma;

   private:
    Reward xi, eta;
    float  epsilon;

   protected:
    // clang-format off
    typedef std::unordered_map<
        State,
        std::map<
            Action,
            Reward
        >,
        utils::reference_wrapper::hash    <State::type>,
        utils::reference_wrapper::equal_to<State::type>
    > QMatrixType;
    // clang-format on

   private:
    /**
     * TODO: ideas:
     * - Merge all actions ending at state s into one Q-value (making it
     *   practically very similar to a shortest-path problem)
     */
    mutable QMatrixType QMatrix;
    // mutable std::unordered_map<
    //     State,
    //     Reward,
    //     utils::reference_wrapper::hash    <State::type>,
    //     utils::reference_wrapper::equal_to<State::type>
    // > QMatrix;

   protected:
    std::optional<std::reference_wrapper<Logger>> policyLogger;

    virtual Reward estimateOptimalValue(const State& s) const;
    virtual Reward estimateOptimalFutureValue(const State& s, const Action& a) const;

    virtual Action heuristicPolicy(const State& s) const;
    virtual Reward heuristic(const State& s, const Action& a) const;

    virtual Reward tabu(const State& s, const Action& a, const Env::Vehicle& vehicle) const;

    virtual Reward& Qref(const State& s, const Action& a);
    virtual Reward  Q(const State& s, const Action& a) const;

    virtual void updateMatrix(const State& s, const Action& a, Reward reward);

    virtual Reward estimateInitialValue(const State& s, const Action& a) const;

   public:
    QLearner(
        Env::Env&                                               env,
        const SUMO::Network&                                    network,
        const Dynamic::SUMOAdapter&                             adapter,
        const Env::TAZ&                                         destinationTAZ,
        std::optional<std::reference_wrapper<QLearner::Logger>> policyLogger = std::nullopt,
        Reward                                                  alpha        = 0.5,
        Reward                                                  gamma        = 1.0,
        Reward                                                  xi           = 0.0,
        Reward                                                  eta          = 1.0,
        float                                                   epsilon      = 1.0e-3f
    );

    void setAlpha(Reward alpha);
    void setEpsilon(float epsilon);

    void dump() const;
};
}  // namespace Dynamic
