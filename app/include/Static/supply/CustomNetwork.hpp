#pragma once

#include <functional>

#include "Static/supply/Network.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Static {
class CustomNetwork: public Network {
   public:
    typedef std::function<Time(Flow)> CostFunction;

    struct Edge: public Network::Edge {
        friend CustomNetwork;

        CostFunction cost;
        CostFunction costGlobal;

       private:
        Edge(ID id, Node u, Node v, CostFunction f, CostFunction fGlobal);

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;

        virtual ~Edge(){}
    };

   private:
    std::unordered_map<Node, std::vector<Edge *>> adj;
    std::unordered_map<Edge::ID, Edge *>          edges;

   public:
    void addNode(Node u);
    void addEdge(Edge::ID id, Node u, Node v, CostFunction f, CostFunction fGlobal);

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        &getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

    virtual void saveResultsToFile(
        const SUMO::NetworkTAZs &sumo,
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &edgeDataPath,
        const std::string       &routesPath
    ) const;

    virtual ~CustomNetwork();
};
}  // namespace Static
