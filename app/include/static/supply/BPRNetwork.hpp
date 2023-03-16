#pragma once

#include "StaticNetwork.hpp"
#include "data/SumoAdapterStatic.hpp"
#include "data/SumoNetwork.hpp"
#include "data/SumoTAZs.hpp"

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
    std::unordered_map<Edge::Id, CustomEdge *> edges;

    double alpha, beta;

   public:
    BPRNetwork(double alpha = 0.15, double beta = 4.0);

    void addNode(Node u);
    void addEdge(Edge::Id id, Node u, Node v, Time t0, Capacity c);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;

    virtual Cost calculateCost(Edge::Id id, Flow f) const;
    Cost calculateCongestion(Edge::Id id, Flow f) const;

    static std::pair<
        BPRNetwork *,
        SumoAdapterStatic
    > fromSumo(const SumoNetwork &sumoNetwork, const SumoTAZs &sumoTAZs);

    void saveResultsToFile(
        const StaticSolution &x,
        const SumoAdapterStatic &adapter,
        const std::string &path) const;
};
