#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Alg/Graph.hpp"
#include "Static/supply/Network.hpp"

class SumoAdapterStatic;

namespace Static {
class Solution;

class NetworkDifferentiable: public Network {
   public:
    struct Edge: public Network::Edge {
       protected:
        Edge(Edge::ID id, Node u, Node v);

       public:
        virtual Cost calculateCostDerivative(const Solution &x) const = 0;
    };
    virtual Edge *getEdge(Edge::ID e) const = 0;
};

}  // namespace Static
