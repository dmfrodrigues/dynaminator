#pragma once

#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"

namespace Static {
class BPRConvexNetwork: public NetworkDifferentiable {
    class Edge;
    friend class Edge;

    const BPRNotConvexNetwork &bprNotConvex;
    const Solution &solution;

    class Edge: public NetworkDifferentiable::Edge {
        friend class BPRConvexNetwork;

        const BPRConvexNetwork &bprConvex;
        const BPRNotConvexNetwork &bprNotConvex;
        Edge::ID id;

        Edge(
            const BPRConvexNetwork &bprConvexNetwork_,
            const BPRNotConvexNetwork &bprNotConvex_,
            Edge::ID id_
        );

       public:
        virtual Cost calculateCost(const Solution &x) const;
        virtual Cost calculateCostGlobal(const Solution &x) const;
        virtual Cost calculateCostDerivative(const Solution &x) const;
    };

   public:
    BPRConvexNetwork(const BPRNotConvexNetwork &bprNotConvex_, const Solution &solution_);

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        *getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

    virtual void saveResultsToFile(
        const SUMO::NetworkTAZs &sumo,
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &edgeDataPath,
        const std::string       &routesPath
    ) const;
};
}  // namespace Static
