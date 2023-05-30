#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <tuple>

#include "Alg/Flow/EdmondsKarp.hpp"
#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/BFS.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace Static;
using namespace utils;

using Alg::Graph;

typedef BPRNotConvexNetwork::Time Time;
typedef BPRNotConvexNetwork::Flow Flow;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;
typedef SUMO::Length              Length;

const Flow SATURATION_FLOW             = 1110.0;  // vehicles per hour per lane

/**
 * @brief Cost of having traffic stop once. This should be a time penalty that
 * averages all the negative effects of cars having to start after the light
 * turns green.
 */
const Time STOP_PENALTY = 0.0;

BPRNotConvexNetwork *BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::load(const SUMO::NetworkTAZs &sumo) {
    clear();

    networkNotConvex = new BPRNotConvexNetwork();
    network = networkNotConvex;

    addNormalEdges(sumo);

    addConnections(sumo);

    iterateCapacities(sumo);

    addDeadEnds(sumo);

    addTAZs(sumo);

    for(auto &[eID, _]: connectionEdges) {
        addConnectionConflicts(sumo, eID);
    }

    return networkNotConvex;
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to) {
    auto fromToConnections = sumo.network.getConnections(from, to);

    if(fromToConnections.empty()) return;

    Speed v = min(
        calculateFreeFlowSpeed(from),
        calculateFreeFlowSpeed(to)
    );
    Time adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (v / calculateFreeFlowSpeed(50.0 / 3.6));

    vector<Time> capacityFromLanes(from.lanes.size(), 0.0);
    vector<Time> capacityToLanes(to.lanes.size(), 0.0);

    double t0 = 0;

    for(const SUMO::Network::Connection *connPtr: fromToConnections) {
        const SUMO::Network::Connection &conn = *connPtr;

        Time cAdd = adjSaturationFlow;
        /// Traffic lights
        if(conn.tl) {
            SUMO::Time g = conn.getGreenTime();
            SUMO::Time C = conn.getCycleTime();
            SUMO::Time r = C - g;
            size_t     n = conn.getNumberStops();
            cAdd *= (g - STOP_PENALTY * (double)n) / C;

            t0 += r * r / (2.0 * C);
        }
        capacityFromLanes[conn.fromLane().index] += cAdd;
        capacityToLanes[conn.toLane().index] += cAdd;

        /// Direction changes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
        // clang-format off
        switch(conn.dir) {
            case SUMO::Network::Connection::Direction::PARTIALLY_RIGHT: t0 += 1.0; break;
            case SUMO::Network::Connection::Direction::RIGHT          : t0 += 2.0; break;
            case SUMO::Network::Connection::Direction::PARTIALLY_LEFT : t0 += 2.0; break;
            case SUMO::Network::Connection::Direction::LEFT           : t0 += 5.0; break;
            default:
                break;
        }
            // clang-format on
#pragma GCC diagnostic pop
    }
    t0 /= (double)fromToConnections.size();

    for(Time &c: capacityFromLanes) c = min(c, adjSaturationFlow);
    for(Time &c: capacityToLanes) c = min(c, adjSaturationFlow);
    double cFrom = accumulate(capacityFromLanes.begin(), capacityFromLanes.end(), 0.0);
    double cTo   = accumulate(capacityToLanes.begin(), capacityToLanes.end(), 0.0);
    double c     = min(cFrom, cTo);

    BPRNetwork::ConnectionEdge *e = network->addConnectionEdge(
        adapter.addEdge(),
        adapter.toNodes(from.id).second,
        adapter.toNodes(to.id).first,
        *network,
        t0,
        c
    );

    connectionEdges[e->id] = make_tuple(e, from.id, to.id);

    connectionMap[from.id][to.id] = e->id;
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addConnectionConflicts(const SUMO::NetworkTAZs &sumo, const Edge::ID &eID) {
    auto &[e_, fromID, toID] = connectionEdges.at(eID);
    ConnectionEdge *e        = dynamic_cast<ConnectionEdge*>(e_);
    assert(e != nullptr);

    const SUMO::Network::Edge &from = sumo.network.getEdge(fromID);
    const SUMO::Network::Edge &to   = sumo.network.getEdge(toID);

    const vector<const SUMO::Network::Connection *> &fromToConnections = sumo.network.getConnections(from, to);

    for(const SUMO::Network::Connection *connPtr: fromToConnections) {
        const SUMO::Network::Connection &conn = *connPtr;

        if(conn.tl) {
            e->conflicts.clear();
            return;
        }

        vector<const SUMO::Network::Connection *> response = conn.getRequest().getResponse();

        vector<pair<const Edge *, double>> conf;

        for(const SUMO::Network::Connection *rPtr: response) {
            const SUMO::Network::Connection &r = *rPtr;

            ConnectionEdge::ID cID = connectionMap.at(r.from.id).at(r.to.id);
            ConnectionEdge *c = dynamic_cast<ConnectionEdge*>(get<0>(connectionEdges.at(cID)));
            assert(c != nullptr);

            size_t n = getNumberLanes(sumo, *c);
            assert(n > 0);

            conf.push_back({c, 1.0 / (double)n});
        }

        e->conflicts.push_back(conf);
    }
}

size_t BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::getNumberLanes(const SUMO::NetworkTAZs &sumo, const ConnectionEdge &e) const {
    const SUMO::Network::Edge::ID &fromID = adapter.fromNodeToSumoEdge(e.u);
    const SUMO::Network::Edge::ID &toID   = adapter.fromNodeToSumoEdge(e.v);

    vector<const SUMO::Network::Connection *> connections = sumo.network.getConnections(
        sumo.network.getEdge(fromID),
        sumo.network.getEdge(toID)
    );
    set<SUMO::Index> fromLanes;
    set<SUMO::Index> toLanes;
    for(const SUMO::Network::Connection *connection: connections) {
        fromLanes.insert(connection->fromLane().index);
        toLanes.insert(connection->toLane().index);
    }
    return min(fromLanes.size(), toLanes.size());
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::clear() {
    BPRNetwork::Loader<SUMO::NetworkTAZs>::clear();
    connectionMap.clear();
}
