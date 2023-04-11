#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Graph.hpp"

class SumoAdapterStatic;

namespace Static {
class Solution;

class Network {
   public:
    typedef long   Node;
    typedef double Flow;
    typedef double Cost;

    struct Edge {
        typedef long ID;
        ID           id;
        Node         u, v;

       protected:
        Edge(ID id, Node u, Node v);

       public:
        virtual Cost calculateCost(const Solution &x) const       = 0;
        virtual Cost calculateCostGlobal(const Solution &x) const = 0;
    };

    typedef std::vector<Edge::ID> Path;

    virtual std::vector<Node>   getNodes() const          = 0;
    virtual Edge               *getEdge(Edge::ID e) const = 0;
    virtual std::vector<Edge *> getAdj(Node u) const      = 0;

    Graph toGraph(const Solution &solution) const;

    Cost evaluate(const Solution &solution) const;

    virtual void saveResultsToFile(
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &edgeDataPath,
        const std::string       &routesPath
    ) const = 0;

    virtual ~Network() {}
};
}  // namespace Static

namespace std {
template<>
struct hash<Static::Network::Path> {
    std::size_t operator()(const Static::Network::Path &v) const;
};
}  // namespace std