#pragma once

#include <functional>

#include "data/sumo/Network.hpp"
#include "data/sumo/TAZs.hpp"
#include "static/supply/StaticNetwork.hpp"

class CustomStaticNetwork: public StaticNetwork {
   public:
    typedef std::function<Cost(Flow)> CostFunction;

   private:
    struct CustomEdge: public Edge {
        CostFunction cost;
        CostFunction costGlobal;
    };

    std::unordered_map<Node, std::vector<CustomEdge *>> adj;
    std::unordered_map<Edge::ID, CustomEdge *> edges;

   public:
    void addNode(Node u);
    void addEdge(Edge::ID id, Node u, Node v, CostFunction f, CostFunction fGlobal);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;
    virtual Cost calculateCost(Edge::ID id, Flow f) const;
    virtual Cost calculateCostGlobal(Edge::ID id, Flow f) const;

    virtual void saveResultsToFile(
        const StaticSolution &x,
        const SumoAdapterStatic &adapter,
        const std::string &edgeDataPath,
        const std::string &routesPath
    ) const;

    ~CustomStaticNetwork();
};
