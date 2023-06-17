#pragma once

#include <map>
#include <memory>
#include <random>

#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/ShortestPathManyMany.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic {

/**
 * @brief Policy that follows a predefined path.
 *
 * This policy does not provide any margin for improvement, as it ignores all
 * feedback and always follows the same path that the policy was constructed
 * with.
 */
class PathPolicy: public Vehicle::Policy {
    static const Env::Edge::ID END = -1;

    Vehicle::ID id;

    std::unordered_map<Env::Edge::ID, Env::Edge::ID> nextEdgeMap;

    std::shared_ptr<std::mt19937> gen;

   public:
    PathPolicy(
        Vehicle::ID                   id,
        const Alg::Graph::Path       &path,
        std::shared_ptr<std::mt19937> gen
    );

    virtual Vehicle::Policy::Intention pickConnection(
        const Env::Env &env
    ) override;

    virtual void feedback(
        const Env::Edge &e,
        Time             t
    ) override;

    /**
     * @brief Shortest-path policy factory.
     *
     * This factory creates instances of PathPolicy using the paths generated
     * by a shortest-path algorithm. As such, vehicles using policies from this
     * factory are expected to use the shortest paths between origins and
     * destinations, while ignoring delays owed to congestion.
     */
    class ShortestPathFactory: public Vehicle::Policy::Factory {
        const Alg::ShortestPath::ShortestPathManyMany &sp;

        std::shared_ptr<std::mt19937> gen;

       public:
        ShortestPathFactory(
            const Alg::ShortestPath::ShortestPathManyMany &sp
        );
        ShortestPathFactory(
            const Alg::ShortestPath::ShortestPathManyMany &sp,
            std::random_device::result_type                seed
        );
        virtual std::shared_ptr<Policy> create(
            Vehicle::ID      id,
            Time             depart,
            const Env::Edge &from,
            const Env::Edge &to
        ) override;
    };
};

}  // namespace Dynamic
