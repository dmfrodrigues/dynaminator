#pragma once

#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Static {
class BPRNetwork: public NetworkDifferentiable {
   public:
    typedef double Time;
    typedef double Capacity;

    struct Edge: public NetworkDifferentiable::Edge {
       public:
        Edge(ID id, Node u, Node v);
    };

    struct NormalEdge: public Edge {
        friend BPRNetwork;

        const BPRNetwork &network;

        Time     t0;
        Capacity c;

       private:
        NormalEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Capacity c);

       public:
        virtual Cost calculateCost(const Solution &x) const;
        virtual Cost calculateCostGlobal(const Solution &x) const;
        virtual Cost calculateCostDerivative(const Solution &x) const;

        Cost calculateCongestion(const Solution &x) const;
    };
    struct ConnectionEdge: public Edge {
        friend BPRNetwork;

        const BPRNetwork &network;

        Time     t0;
        Capacity c;

       private:
        ConnectionEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Capacity c);

       public:
        virtual Cost calculateCost(const Solution &x) const;
        virtual Cost calculateCostGlobal(const Solution &x) const;
        virtual Cost calculateCostDerivative(const Solution &x) const;
    };

   private:
    std::unordered_map<Node, std::vector<Edge *>> adj;
    std::unordered_map<Edge::ID, Edge *>          edges;

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
    void addEdge(Edge *e);

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        *getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

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
