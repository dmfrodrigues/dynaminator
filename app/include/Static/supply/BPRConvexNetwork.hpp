#pragma once

#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"

namespace Static {
class BPRConvexNetwork: public NetworkDifferentiable {
   public:
    struct Edge;

   private:
    friend struct Edge;
    const BPRNotConvexNetwork &bprNotConvex;
    const Solution            &solution;

   public:
    struct Edge: public NetworkDifferentiable::Edge {
        friend class BPRConvexNetwork;

        const BPRConvexNetwork    &bprConvex;
        const BPRNotConvexNetwork &bprNotConvex;

        Edge(
            const BPRConvexNetwork    &bprConvexNetwork_,
            const BPRNotConvexNetwork &bprNotConvex_,
            Edge::ID                   id_
        );

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;
        virtual Time calculateCostDerivative(const Solution &x) const;
    };

   private:
    mutable std::unordered_map<Edge::ID, Edge> edges;

   public:
    BPRConvexNetwork(const BPRNotConvexNetwork &bprNotConvex_, const Solution &solution_);

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        &getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;
};
}  // namespace Static
