#pragma once

#include <map>
#include <memory>
#include <random>

#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/ShortestPathManyMany.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic {

class PathPolicy: public Vehicle::Policy {
    static const Env::Edge::ID END = -1;

    Vehicle::ID id;

    std::unordered_map<Env::Edge::ID, Env::Edge::ID> nextEdge;

    std::shared_ptr<std::mt19937> gen;

   public:
    PathPolicy(
        Vehicle::ID                   id,
        const Alg::Graph::Path       &path,
        std::shared_ptr<std::mt19937> gen
    );

    virtual const Env::Connection &pickConnection(
        const Env::Env &env
    ) override;

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
