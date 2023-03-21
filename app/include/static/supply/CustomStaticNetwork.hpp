#pragma once

#include <functional>

#include "static/supply/StaticNetwork.hpp"
#include "data/SumoNetwork.hpp"
#include "data/SumoTAZs.hpp"

class CustomStaticNetwork : public StaticNetwork {
   public:
    typedef std::function<Cost(Flow)> CostFunction;

   private:
    struct CustomEdge : public Edge {
        CostFunction cost;
    };

    std::unordered_map<Node, std::vector<CustomEdge *>> adj;
    std::unordered_map<Edge::Id, CustomEdge *> edges;

   public:
    void addNode(Node u);
    void addEdge(Edge::Id id, Node u, Node v, CostFunction f);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;
    virtual Cost calculateCost(Edge::Id id, Flow f) const;

    virtual void saveResultsToFile(const StaticSolution &x,
                                   const SumoAdapterStatic &adapter,
                                   const std::string &path) const;

    ~CustomStaticNetwork();
};
