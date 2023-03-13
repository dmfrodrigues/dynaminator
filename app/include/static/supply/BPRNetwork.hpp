#pragma once

#include <tuple>

#include "StaticNetwork.hpp"
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

    std::unordered_map<Node, std::vector<CustomEdge*>> adj;
    std::unordered_map<Edge::Id, CustomEdge*> edges;

    double alpha, beta;

   public:
    BPRNetwork(double alpha = 0.15, double beta = 4.0);

    void addNode(Node u);
    void addEdge(Edge::Id id, Node u, Node v, Time t0, Capacity c);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge*> getAdj(Node u) const;

    virtual Cost calculateCost(Edge::Id id, Flow f) const;
    Cost calculateCongestion(Edge::Id id, Flow f) const;

    static std::tuple<
        BPRNetwork*,
        std::unordered_map<SumoNetwork::Junction::Id, Node>,
        std::unordered_map<SumoNetwork::Junction::Id, std::pair<Node, Node>>,
        std::unordered_map<SumoNetwork::Edge::Id, Edge::Id>
    > fromSumo(const SumoNetwork& sumoNetwork, const SumoTAZs& sumoTAZs);

    void saveResultsToFile(
        const StaticSolution &x,
        const std::unordered_map<
            SumoNetwork::Edge::Id,
            StaticNetwork::Edge::Id
        > &edgeStr2id,
        const std::string &path
    ) const;
};
