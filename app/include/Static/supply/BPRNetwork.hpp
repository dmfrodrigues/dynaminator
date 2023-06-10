#pragma once

#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/TAZ.hpp"
#include "Static/SUMOAdapter.hpp"

namespace Static {
class BPRNetwork: public NetworkDifferentiable {
   public:
    template<typename T>
    class Loader {
       public:
        BPRNetwork *load(const T &t);
    };

    struct Edge: public NetworkDifferentiable::Edge {
        template<typename T>
        friend class Loader;

        friend BPRNetwork;

        const BPRNetwork &network;

        Time t0;
        Flow c;

       protected:
        Edge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);
        virtual ~Edge();

       public:
        Time calculateCongestion(const Solution &x) const;
    };

    struct NormalEdge: public Edge {
        template<typename T>
        friend class Loader;

        friend BPRNetwork;

       protected:
        NormalEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;
        virtual Time calculateCostDerivative(const Solution &x) const;
    };
    struct ConnectionEdge: public Edge {
        template<typename T>
        friend class Loader;

        friend BPRNetwork;

       protected:
        ConnectionEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;
        virtual Time calculateCostDerivative(const Solution &x) const;
    };

   protected:
    std::map<Node, std::vector<Edge *>> adj;
    std::map<Edge::ID, Edge *>          edges;

   public:
    BPRNetwork(Time alpha = 0.15, Time beta = 4.0);

    virtual void addNode(Node u);
    virtual NormalEdge *addNormalEdge(
        Edge::ID id,
        Node u,
        Node v,
        const BPRNetwork &network,
        Time t0,
        Flow c
    );
    virtual ConnectionEdge *addConnectionEdge(Edge::ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);

    const Time alpha, beta;

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        &getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

    ~BPRNetwork();
};

template<>
class BPRNetwork::Loader<SUMO::NetworkTAZs> {
   protected:
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

    virtual Time calculateFreeFlowSpeed(const Time &maxSpeed) const;
    virtual Time calculateFreeFlowSpeed(const SUMO::Network::Edge &e) const;
    virtual Time calculateFreeFlowSpeed(const SUMO::Network::Edge::Lane &l) const;
    virtual Time calculateFreeFlowTime(const SUMO::Network::Edge &e) const;
    virtual Time calculateFreeFlowTime(const SUMO::Network::Edge::Lane &l) const;
    virtual Flow calculateCapacity(const SUMO::Network::Edge &e) const;
    virtual Flow calculateCapacity(const SUMO::Network::Edge::Lane &l) const;

    virtual void addNormalEdges(const SUMO::NetworkTAZs &sumo);
    virtual void addConnections(const SUMO::NetworkTAZs &sumo);
    virtual void iterateCapacities(const SUMO::NetworkTAZs &sumo);
    virtual void addTAZs(const SUMO::NetworkTAZs &sumo);

    virtual void addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to);

   public:
    SUMOAdapter adapter;

    virtual void clear();

    virtual BPRNetwork *load(const SUMO::NetworkTAZs &sumo);

    virtual ~Loader();
};

}  // namespace Static
