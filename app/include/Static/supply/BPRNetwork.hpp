#pragma once

#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/TAZ.hpp"
#include "data/SumoAdapterStatic.hpp"

namespace Static {
class BPRNetwork: public NetworkDifferentiable {
   public:
    typedef double Time;
    typedef double Capacity;

    template<typename T>
    class Loader {
       public:
        BPRNetwork *load(const T &t);
    };

    struct Edge: public NetworkDifferentiable::Edge {
        template<typename T>
        friend class Loader;

        Capacity c;

       protected:
        Edge(ID id, Node u, Node v, Capacity c);
    };
    typedef std::unordered_map<Edge::ID, Edge *> Edges;

    struct NormalEdge: public Edge {
        template<typename T>
        friend class Loader;

        const BPRNetwork &network;

        Time t0;

       protected:
        NormalEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Capacity c);

       public:
        virtual Cost calculateCost(const Solution &x) const;
        virtual Cost calculateCostGlobal(const Solution &x) const;
        virtual Cost calculateCostDerivative(const Solution &x) const;

        Cost calculateCongestion(const Solution &x) const;
    };
    struct ConnectionEdge: public Edge {
        template<typename T>
        friend class Loader;

        const BPRNetwork &network;

        Time t0;

       protected:
        ConnectionEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Capacity c);

       public:
        virtual Cost calculateCost(const Solution &x) const;
        virtual Cost calculateCostGlobal(const Solution &x) const;
        virtual Cost calculateCostDerivative(const Solution &x) const;
    };

   private:
    typedef std::unordered_map<Node, std::vector<Edge *>> Adj;

    Adj   adj;
    Edges edges;

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
    virtual Edge                        &getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

    virtual void saveResultsToFile(
        const SUMO::NetworkTAZs &sumo,
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &edgeDataPath,
        const std::string       &routesPath
    ) const;
};

template<>
class BPRNetwork::Loader<SUMO::NetworkTAZs> {
    BPRNetwork *network;

    std::map<SUMO::Network::Junction::ID, std::list<SUMO::Network::Edge>> in, out;

    std::map<SUMO::Network::Edge::ID, NormalEdge *> normalEdges;

    // clang-format off
    std::map<
        ConnectionEdge::ID,
        std::tuple<
            ConnectionEdge *,
            SUMO::Network::Edge::ID,
            SUMO::Network::Edge::ID
        >
    > connectionEdges;
    // clang-format on

    Cost     calculateFreeFlowSpeed(const Cost &maxSpeed) const;
    Cost     calculateFreeFlowSpeed(const SUMO::Network::Edge &e) const;
    Cost     calculateFreeFlowSpeed(const SUMO::Network::Edge::Lane &l) const;
    Cost     calculateFreeFlowTime(const SUMO::Network::Edge &e) const;
    Cost     calculateFreeFlowTime(const SUMO::Network::Edge::Lane &l) const;
    Capacity calculateCapacity(const SUMO::Network::Edge &e) const;
    Capacity calculateCapacity(const SUMO::Network::Edge::Lane &l) const;

    void addNormalEdges(const SUMO::NetworkTAZs &sumo);
    void addConnections(const SUMO::NetworkTAZs &sumo);
    void iterateCapacities(const SUMO::NetworkTAZs &sumo);
    void addDeadEnds(const SUMO::NetworkTAZs &sumo);
    void addTAZs(const SUMO::NetworkTAZs &sumo);

    void addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to);

   public:
    SumoAdapterStatic adapter;

    void clear();

    BPRNetwork *load(const SUMO::NetworkTAZs &sumo);
};

}  // namespace Static
