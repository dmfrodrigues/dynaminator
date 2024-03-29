#pragma once

#include "Static/SUMOAdapter.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/TAZ.hpp"

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

        friend BPRNotConvexNetwork;

       protected:
        NormalEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);
    };
    struct ConnectionEdge: public BPRNetwork::ConnectionEdge {
        template<typename T>
        friend class BPRNotConvexNetwork::Loader;

        friend BPRNotConvexNetwork;

        std::vector<std::vector<std::pair<const Edge *, double>>> conflicts;

       protected:
        ConnectionEdge(ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);

       private:
        Time getLessPriorityCapacity(const Solution &x) const;

       public:
        virtual Time calculateCost(const Solution &x) const;
        virtual Time calculateCostGlobal(const Solution &x) const;
        virtual Time calculateCostDerivative(const Solution &x) const;
    };

   public:
    BPRNotConvexNetwork(Time alpha = 0.15, Time beta = 4.0);

    virtual NormalEdge     *addNormalEdge(Edge::ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);
    virtual ConnectionEdge *addConnectionEdge(Edge::ID id, Node u, Node v, const BPRNetwork &network, Time t0, Flow c);

    BPRConvexNetwork makeConvex(const Solution &x) const;
};

template<>
class BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>: public BPRNetwork::Loader<SUMO::NetworkTAZs> {
    BPRNotConvexNetwork *networkNotConvex;

    // clang-format off
    std::map<
        SUMO::Network::Edge::ID,
        std::map<
            SUMO::Network::Edge::ID,
            ConnectionEdge::ID
        >
    > connectionMap;
    // clang-format on

    void addConnectionConflicts(const SUMO::NetworkTAZs &sumo, const Edge::ID &eID);

    size_t getNumberLanes(const SUMO::NetworkTAZs &sumo, const ConnectionEdge &e) const;

   public:
    virtual void clear();

    virtual BPRNotConvexNetwork *load(const SUMO::NetworkTAZs &sumo);
};

}  // namespace Static
