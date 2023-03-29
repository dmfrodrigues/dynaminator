#pragma once

#include "static/supply/StaticNetwork.hpp"
#include "data/sumo/Network.hpp"
#include "data/sumo/TAZs.hpp"

class BPRNetwork : public StaticNetwork {
   public:
    typedef double Time;
    typedef double Capacity;

   private:
    struct CustomEdge : public Edge {
        Time t0;
        Capacity c;
    };

    std::unordered_map<Node, std::vector<CustomEdge *>> adj;
    std::unordered_map<Edge::ID, CustomEdge *> edges;

    StaticNetwork::Flow alpha, beta;

   public:
    BPRNetwork(StaticNetwork::Flow alpha = 0.15, StaticNetwork::Flow beta = 4.0);

    void addNode(Node u);
    void addEdge(Edge::ID id, Node u, Node v, Time t0, Capacity c);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;

    virtual Cost calculateCost(Edge::ID id, Flow f) const;
    virtual Cost calculateCostGlobal(Edge::ID id, Flow f) const;

    Cost calculateCongestion(Edge::ID id, Flow f) const;
    Cost calculateDelay(Edge::ID id, Flow f) const;

    static std::pair<
        BPRNetwork *,
        SumoAdapterStatic
    > fromSumo(const SUMO::Network &sumoNetwork, const SumoTAZs &sumoTAZs);

    virtual void saveResultsToFile(
        const StaticSolution &x,
        const SumoAdapterStatic &adapter,
        const std::string &path) const;
};
