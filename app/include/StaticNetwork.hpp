#pragma once

#include <unordered_map>
#include <vector>

#include "Graph.hpp"

class StaticSolution;

class StaticNetwork {
   public:
    typedef long Node;
    typedef double Flow;
    typedef double Cost;

    struct Edge {
        typedef long Id;
        Id id;
        Node v;
    };

    class Path : public std::vector<Edge::Id> {};

    virtual const std::vector<Node> &getNodes() const = 0;
    virtual const std::vector<Edge *> &getAdj(Node u) const = 0;

    virtual Cost calculateCost(Edge::Id id, Flow f) const = 0;

    Graph toGraph(const StaticSolution &solution) const;

    virtual Cost evaluate(const StaticSolution &solution) const = 0;
};

namespace std {
template <> struct hash<StaticNetwork::Path> {
    std::size_t operator()(const StaticNetwork::Path &v) const;
};
}
