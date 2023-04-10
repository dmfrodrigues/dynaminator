#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Graph.hpp"

class StaticSolution;

class SumoAdapterStatic;

class StaticNetwork {
   public:
    typedef long Node;
    typedef double Flow;
    typedef double Cost;

    struct Edge {
        typedef long ID;
        ID id;
        Node u, v;
    };

    typedef std::vector<Edge::ID> Path;

    virtual std::vector<Node> getNodes() const = 0;
    virtual std::vector<Edge *> getAdj(Node u) const = 0;

    virtual Cost calculateCost(Edge::ID id, Flow f) const = 0;
    virtual Cost calculateCostGlobal(Edge::ID id, Flow f) const = 0;

    Graph toGraph(const StaticSolution &solution) const;

    Cost evaluate(const StaticSolution &solution) const;

    virtual void saveResultsToFile(
        const StaticSolution &x,
        const SumoAdapterStatic &adapter,
        const std::string &edgeDataPath,
        const std::string &routesPath
    ) const = 0;

    virtual ~StaticNetwork() {}
};

namespace std {
template<>
struct hash<StaticNetwork::Path> {
    std::size_t operator()(const StaticNetwork::Path &v) const;
};
}  // namespace std
