#pragma once

#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Static {
class BPRNetwork: public NetworkDifferentiable {
   public:
    typedef double Time;
    typedef double Capacity;

   private:
    struct CustomEdge: public Edge {
        Time     t0;
        Capacity c;
    };

    std::unordered_map<Node, std::vector<CustomEdge *>> adj;
    std::unordered_map<Edge::ID, CustomEdge *>          edges;

    Network::Flow alpha, beta;

    void saveEdges(
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &path
    ) const;

    void saveRoutes(
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &path
    ) const;

   public:
    BPRNetwork(Network::Flow alpha = 0.15, Network::Flow beta = 4.0);

    void addNode(Node u);
    void addEdge(CustomEdge *e);

    virtual std::vector<Node>   getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;

    virtual Cost calculateCost(Edge::ID id, Flow f) const;
    virtual Cost calculateCostGlobal(Edge::ID id, Flow f) const;
    virtual Cost calculateCostDerivative(Edge::ID id, Flow f) const;

    Cost calculateCongestion(Edge::ID id, Flow f) const;

    static std::pair<
        BPRNetwork *,
        SumoAdapterStatic>
    fromSumo(const SUMO::Network &sumoNetwork, const SUMO::TAZs &sumoTAZs);

    virtual void saveResultsToFile(
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &edgeDataPath,
        const std::string       &routesPath
    ) const;
};
}  // namespace Static
