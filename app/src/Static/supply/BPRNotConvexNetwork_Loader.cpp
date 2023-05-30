#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>
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

    for(const auto &[eID, t]: connectionEdges){
        const auto &[e, fromID, toID] = t;
        if(connectionMap.count(fromID) && connectionMap.at(fromID).count(toID)){
            throw logic_error("BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::load: Connection already exists in connectionMap");
        }
        connectionMap[fromID][toID] = eID;
    }

    for(auto &[eID, _]: connectionEdges) {
        addConnectionConflicts(sumo, eID);
    }

    return networkNotConvex;
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
