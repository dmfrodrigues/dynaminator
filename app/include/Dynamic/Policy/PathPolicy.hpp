#pragma once

#include <map>
#include <memory>
#include <random>

#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Alg/ShortestPath/ShortestPathManyMany.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Policy/Policy.hpp"

namespace Dynamic {

/**
 * @brief Policy that follows a predefined path.
 *
 * This policy does not provide any margin for improvement, as it ignores all
 * feedback and always follows the same path that the policy was constructed
 * with.
 */
class PathPolicy: public Policy {
   public:
    struct Action: public Env::Action {
        Action(Env::Connection &connection, Env::Lane &lane);

        virtual void reward(Reward r) override;
    };

   private:
    static const Env::Edge::ID START;
    static const Env::Edge::ID END;

    Vehicle::ID id;

    std::unordered_map<Env::Edge::ID, Env::Edge::ID> nextEdgeMap;

    std::shared_ptr<std::mt19937> gen;

   public:
    PathPolicy(
        Vehicle::ID                   id,
        const Alg::Graph::Path       &path,
        std::shared_ptr<std::mt19937> gen
    );

    virtual Env::Lane &pickInitialLane(
        Vehicle  &vehicle,
        Env::Env &env
    ) override;

    virtual std::shared_ptr<Env::Action> pickConnection(
        Env::Env &env
    ) override;

    /**
     * @brief Shortest-path policy factory.
     *
     * This factory creates instances of PathPolicy using the paths generated
     * by a shortest-path algorithm. As such, vehicles using policies from this
     * factory are expected to use the shortest paths between origins and
     * destinations, while ignoring delays owed to congestion.
     */
    class ShortestPathFactory: public Policy::Factory {
        Alg::ShortestPath::DijkstraMany sp;

        std::shared_ptr<std::mt19937> gen;

       public:
        ShortestPathFactory(const Env::Env &env);
        ShortestPathFactory(const Env::Env &env, std::random_device::result_type seed);
        virtual std::shared_ptr<Policy> create(
            Vehicle::ID     id,
            Time            depart,
            const Env::TAZ &fromTAZ,
            const Env::TAZ &toTAZ
        ) override;
    };
};

}  // namespace Dynamic
