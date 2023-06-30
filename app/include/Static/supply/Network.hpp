#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Alg/Graph.hpp"
#include "Static/Static.hpp"
#include "data/SUMO/NetworkTAZ.hpp"

namespace Static {
class Solution;

class SUMOAdapter;

class Network {
   public:
    typedef long Node;

    struct Edge {
        typedef long ID;
        ID           id;
        Node         u, v;

       protected:
        Edge(ID id, Node u, Node v);

       public:
        virtual Time calculateCost(const Solution &x) const       = 0;
        virtual Time calculateCostGlobal(const Solution &x) const = 0;

        Time calculateDelay(const Solution &x) const;
    };

    typedef std::vector<Edge::ID> Path;

    virtual std::vector<Node>   getNodes() const          = 0;
    virtual Edge               &getEdge(Edge::ID e) const = 0;
    virtual std::vector<Edge *> getAdj(Node u) const      = 0;

    Alg::Graph toGraph(const Solution &solution) const;

    Time evaluate(const Solution &solution) const;

    virtual ~Network() {}
};
}  // namespace Static

namespace std {
template<>
struct hash<Static::Network::Path> {
    size_t operator()(const Static::Network::Path &v) const;
};
}  // namespace std
