#pragma once

#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/TAZ.hpp"
#include "data/SumoAdapterStatic.hpp"

namespace Static {

class BPRConvexNetwork;

class BPRNotConvexNetwork: public BPRNetwork {
   public:
    template<typename T>
    class Loader {
       public:
        BPRNotConvexNetwork *load(const T &t);
    };

    struct NormalEdge: public BPRNetwork::NormalEdge {
        template<typename T>
        friend class BPRNotConvexNetwork::Loader;

       protected:
        NormalEdge(ID id, Node u, Node v, const BPRNotConvexNetwork &network, Time t0, Flow c);
    };
    struct ConnectionEdge: public Edge {
        template<typename T>
        friend class BPRNotConvexNetwork::Loader;

        std::vector<std::vector<std::pair<const Edge *, double>>> conflicts;

       protected:
        ConnectionEdge(ID id, Node u, Node v, const BPRNotConvexNetwork &network, Time t0, Flow c);

       private:
        Time getLessPriorityCapacity(const Solution &x) const;

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;
        virtual Time calculateCostDerivative(const Solution &x) const;
    };

   private:
    std::map<Node, std::vector<Edge *>> adj;
    std::map<Edge::ID, Edge *>          edges;

   protected:
    virtual void saveEdges(
        const SUMO::NetworkTAZs &sumo,
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &path
    ) const;

    virtual void saveRoutes(
        const Solution          &x,
        const SumoAdapterStatic &adapter,
        const std::string       &path
    ) const;

   private:
    void addNode(Node u);
    void addEdge(Edge *e);

   public:
    BPRNotConvexNetwork(Network::Flow alpha = 0.15, Network::Flow beta = 4.0);

    virtual std::vector<Node>            getNodes() const;
    virtual Edge                        &getEdge(Edge::ID e) const;
    virtual std::vector<Network::Edge *> getAdj(Node u) const;

    BPRConvexNetwork makeConvex(const Solution &x) const;
};

template<>
class BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> {
    BPRNotConvexNetwork *network;

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
    std::map<
        SUMO::Network::Edge::ID,
        std::map<
            SUMO::Network::Edge::ID,
            ConnectionEdge::ID
        >
    > connectionMap;
    // clang-format on

    Time calculateFreeFlowSpeed(const Time &maxSpeed) const;
    Time calculateFreeFlowSpeed(const SUMO::Network::Edge &e) const;
    Time calculateFreeFlowSpeed(const SUMO::Network::Edge::Lane &l) const;
    Time calculateFreeFlowTime(const SUMO::Network::Edge &e) const;
    Time calculateFreeFlowTime(const SUMO::Network::Edge::Lane &l) const;
    Flow calculateCapacity(const SUMO::Network::Edge &e) const;
    Flow calculateCapacity(const SUMO::Network::Edge::Lane &l) const;

    void addNormalEdges(const SUMO::NetworkTAZs &sumo);
    void addConnections(const SUMO::NetworkTAZs &sumo);
    void iterateCapacities(const SUMO::NetworkTAZs &sumo);
    void addDeadEnds(const SUMO::NetworkTAZs &sumo);
    void addTAZs(const SUMO::NetworkTAZs &sumo);

    void addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to);
    void addConnectionConflicts(const SUMO::NetworkTAZs &sumo, const Edge::ID &eID);

    size_t getNumberLanes(const SUMO::NetworkTAZs &sumo, const ConnectionEdge &e) const;

   public:
    SumoAdapterStatic adapter;

    void clear();

    BPRNotConvexNetwork *load(const SUMO::NetworkTAZs &sumo);
};

}  // namespace Static
